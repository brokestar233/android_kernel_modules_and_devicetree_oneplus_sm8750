/******************************************************************************
 ** Copyright (C), 2025-2025, Oplus Mobile Comm Corp., Ltd
 ** File: - oplus_btuart_ux.c
 ** Description: Workqueue UX priority for btuart
 **
 ** Version: 1.0
 ** Date: 2025/01/07
 ** TAG: OPLUS_FEATURE_BTUART_UX
 ** ------------------------------- Revision History: --------------------------
 ** <author>                         <data>        <version>       <desc>
 ** ------------------------------------------------------------------------------
 ******************************************************************************/

#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(6, 6, 0)
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/kprobes.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/tty_port.h>
#include <linux/rcupdate.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <trace/hooks/wqlockup.h>
#include "oplus_btuart_ux.h"
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>

#define VIRTUAL_KWORKER_QMI_NICE (-1003)
#define VIRTUAL_KWORKER_BT_TTY_NICE (-1004)
/* Compare workqueue name with string literal (exact match)
 * Note: This macro should only be used with string literals, not variables
 */
#define WQ_CMP(str)  (strcmp(wq->name, str) == 0)

static struct workqueue_attrs *ux_wq_attrs_qmi;
static struct workqueue_attrs *ux_wq_attrs_bt_tty;

/*
 * Dedicated high-priority workqueue for Bluetooth TTY
 * Protected by RCU for concurrent access
 */
static struct workqueue_struct __rcu *oplus_bt_tty_workqueue;
static DEFINE_MUTEX(bt_tty_wq_mutex);

/* Module state flags */
static bool module_initialized = false;
static bool module_shutting_down = false;
static DEFINE_SPINLOCK(shutdown_lock);

/* Kprobe registration status */
static bool kprobe_tty_flip_buffer_push_registered;
static bool kprobe_tty_buffer_unlock_exclusive_registered;
static bool kprobe_tty_insert_flip_string_and_push_buffer_registered;
static bool kprobe_tty_buffer_restart_work_registered;

/* Dynamic kprobe structures */
static struct kprobe *oplus_tty_flip_buffer_push_kp;
static struct kprobe *oplus_tty_buffer_unlock_exclusive_kp;
static struct kprobe *oplus_tty_insert_flip_string_and_push_buffer_kp;
static struct kprobe *oplus_tty_buffer_restart_work_kp;

static inline int __apply_workqueue_attrs(struct workqueue_struct *wq,
					   const struct workqueue_attrs *attrs)
{
	int ret;

	cpus_read_lock();
	ret = apply_workqueue_attrs(wq, attrs);
	cpus_read_unlock();
	return ret;
}

/* ========== Kprobe Offset Detection ========== */

/**
 * Try to register a kprobe with given offset to verify if it's valid
  * Returns 0 if offset is valid, negative error code otherwise
  */
static int try_register_kprobe_offset(const char *symbol_name,
				       unsigned long offset,
				       kprobe_pre_handler_t handler)
{
	struct kprobe test_kp = {
		.symbol_name = symbol_name,
		.offset = offset,
		.pre_handler = handler,
	};
	int ret;

	pr_info("oplus_btuart_ux: trying kprobe %s offset 0x%lx\n",
		symbol_name, offset);
	ret = register_kprobe(&test_kp);
	if (ret == 0) {
		pr_info("oplus_btuart_ux: kprobe %s offset 0x%lx is valid\n",
			symbol_name, offset);
		unregister_kprobe(&test_kp);
		return 0;  /* offset is valid */
	}

	pr_info("oplus_btuart_ux: kprobe %s offset 0x%lx failed: %d\n",
		symbol_name, offset, ret);
	return ret;  /* offset is invalid */
}

/**
 * Automatically detect and set correct kprobe offset
 * Returns detected offset, or (unsigned long) -1 if all failed
 *
 * Strategy:
 * 1. Try default offset first (usually succeeds)
 * 2. If default fails, search 8 offsets near default (±16 bytes, step 4)
 * 3. This minimizes detection time while maintaining compatibility
 */
static unsigned long detect_kprobe_offset(const char *symbol_name,
					   unsigned long default_offset,
					   kprobe_pre_handler_t handler)
{
	unsigned long detected_offset = (unsigned long) -1;
	unsigned long search_offset;
	int i;

	pr_info("oplus_btuart_ux: detecting kprobe offset for %s (default: 0x%lx)\n",
		symbol_name, default_offset);

	/* First try default offset */
	if (try_register_kprobe_offset(symbol_name, default_offset, handler) == 0) {
		pr_info("oplus_btuart_ux: %s default offset 0x%lx verified successfully\n",
			symbol_name, default_offset);
		return default_offset;
	}

	pr_warn("btuart_ux: %s offset 0x%lx failed, searching...\n",
		symbol_name, default_offset);

	/* Search 8 offsets near default offset (±16 bytes, step 4 bytes)
	 * This covers common variations due to compiler optimizations or
	 * minor kernel version differences
	 * Pattern: -16, -12, -8, -4, +4, +8, +12, +16
	 */
	for (i = 0; i < 8; i++) {
		if (i < 4) {
			/* Negative offsets: -16, -12, -8, -4 */
			unsigned long delta = (unsigned long)(16 - i * 4);
			/* Check for underflow */
			if (default_offset < delta)
				continue;
			search_offset = default_offset - delta;
		} else {
			/* Positive offsets: +4, +8, +12, +16 */
			search_offset = default_offset + (unsigned long)((i - 3) * 4);
		}

		/* Skip if search_offset equals default (already tried) */
		if (search_offset == default_offset)
			continue;

		/* Ensure offset is aligned (must be multiple of 4 for ARM64) */
		if ((search_offset & 0x3) != 0)
			continue;

		pr_info("btuart_ux: %s try offset 0x%lx (delta:%s%lu, %d/8)\n",
			symbol_name, search_offset, (i < 4) ? "-" : "+",
			(unsigned long)((i < 4) ? (16 - i * 4) : ((i - 3) * 4)),
			i + 1);

		if (try_register_kprobe_offset(symbol_name,
					       search_offset,
					       handler) == 0) {
			detected_offset = search_offset;
			pr_info("btuart_ux: %s valid offset 0x%lx (attempt %d/8)\n",
				symbol_name, detected_offset, i + 1);
			break;
		}
	}

	if (detected_offset == (unsigned long) -1) {
		pr_err("btuart_ux: %s offset detect failed near 0x%lx\n",
		       symbol_name, default_offset);
	}

	return detected_offset;
}

/* ========== Bluetooth TTY Port Identification ========== */

/**
 * Check if a TTY port is a Bluetooth port by checking TTY name
 */
static bool is_bluetooth_tty_port(struct tty_struct *tty)
{
	 const char *tty_name_str;

	 if (!tty || !tty->driver) {
		 pr_info("oplus_btuart_ux: is_bluetooth_tty_port: invalid tty or driver\n");
		 return false;
	 }

	/* Method 4: Check TTY name if available */
	tty_name_str = tty->name;
	if (tty_name_str) {
		pr_debug("oplus_btuart_ux: checking TTY name: %s\n", tty_name_str);

		/* For ttyHS, be more conservative: only identify as Bluetooth
		 * if it's ttyHS0 (typically used for Bluetooth on Qualcomm platforms)
		 * This is a fallback when device tree info is not available
		 */
		if (strstr(tty_name_str, "ttyHS")) {
			/* Check if it's ttyHS0 (most common Bluetooth port) */
			if (strcmp(tty_name_str, "ttyHS0") == 0) {
			pr_debug("btuart_ux: BT TTY detected: %s\n",
				 tty_name_str);
				return true;
			}
			/* For other ttyHS ports, log but don't assume Bluetooth */
			pr_debug("btuart_ux: ttyHS %s not ttyHS0, skip\n",
				 tty_name_str);
		}
	}

	pr_debug("oplus_btuart_ux: not a Bluetooth TTY port\n");
	return false;
}

/**
 * Get tty_struct from work_struct
 *
 * Structure relationship:
 *   struct tty_port {
 *       struct tty_bufhead buf;  // contains work_struct work
 *       struct tty_struct *tty;
 *       ...
 *   }
 *
 * We use container_of to get tty_port from work_struct, then get tty_struct.
 * This matches the approach used in flush_to_ldisc() in tty_buffer.c
 */
static struct tty_struct *get_tty_from_work(struct work_struct *work)
{
	struct tty_port *port;
	struct tty_struct *tty;

	if (!work) {
		pr_debug("oplus_btuart_ux: get_tty_from_work: work is NULL\n");
		return NULL;
	}

	port = container_of(work, struct tty_port, buf.work);
	if (!port) {
		pr_debug("oplus_btuart_ux: get_tty_from_work: failed to get tty_port\n");
		return NULL;
	}

	 /* Get tty_struct from tty_port with reference counting
	  * This function returns NULL if tty is not open or port is invalid
	  */
	tty = tty_port_tty_get(port);
	if (!tty) {
		pr_debug("btuart_ux: tty_port_tty_get NULL (not open)\n");
		return NULL;
	}

	pr_debug("oplus_btuart_ux: get_tty_from_work: successfully got tty\n");
	return tty;
}

/* ========== Kprobe Handler ========== */

/**
 * Kprobe handler for tty_buffer queue_work calls
 *
 * ARM64 calling convention at queue_work_on call site:
 *   x0 = first argument (cpu) = WORK_CPU_UNBOUND
 *   x1 = second argument (workqueue pointer) = system_unbound_wq
 *   x2 = third argument (work_struct pointer) = &buf->work
 *
 * We modify x1 to point to our optimized workqueue for Bluetooth ports.
 */
static int handler_tty_buffer_queue_work(struct kprobe *p,
					 struct pt_regs *regs)
{
	struct work_struct *work;
	struct workqueue_struct *target_wq;
	struct tty_struct *tty = NULL;

	 /* Quick check: module shutting down or not initialized */
	if (READ_ONCE(module_shutting_down) || !READ_ONCE(module_initialized)) {
		pr_debug("oplus_btuart_ux: handler skipped (module not ready)\n");
		return 0;
	}

	 /* Get work_struct from register x2 */
	work = (struct work_struct *)regs->regs[2];
	if (!work) {
		pr_debug("oplus_btuart_ux: handler: work is NULL\n");
		return 0;
	}

	pr_debug("oplus_btuart_ux: handler called for work %px\n", work);

	 /* Try to get tty_struct for port filtering */
	tty = get_tty_from_work(work);
	if (tty) {
		/* Check if this is a Bluetooth TTY port */
		if (!is_bluetooth_tty_port(tty)) {
			pr_debug("btuart_ux: not BT TTY, skip redirect\n");
			tty_kref_put(tty);
			return 0;  /* Not a Bluetooth port, don't redirect */
		}
		pr_debug("btuart_ux: BT TTY detected, redirecting\n");
		tty_kref_put(tty);
	} else {
		/* If we can't determine the port (tty not open yet), skip redirect
		 * to avoid misidentifying non-Bluetooth ports. When the tty opens,
		 * subsequent work will be correctly identified and redirected.
		 */
		pr_debug("btuart_ux: tty not open, skip redirect\n");
		return 0;  /* Skip redirect when tty is not available */
	}

	/* Use RCU to safely read workqueue pointer */
	rcu_read_lock();
	target_wq = rcu_dereference(oplus_bt_tty_workqueue);
	if (target_wq) {
		/* Redirect to our high-priority workqueue (modify x1) */
		pr_info("oplus_btuart_ux: handler: redirecting to oplus_bt_tty workqueue\n");
		regs->regs[1] = (u64)target_wq;
	} else {
		/* Workqueue destroyed, don't redirect */
		pr_warn("oplus_btuart_ux: handler: target workqueue not available\n");
	}
	rcu_read_unlock();

	return 0;
}

/* ========== Tracepoint Handlers ========== */

static void android_rvh_alloc_and_link_pwqs_handler(void *unused,
						     struct workqueue_struct *wq, int *ret, bool *skip)
{
	if (WQ_CMP("qmi_msg_handler")) {
		*ret = __apply_workqueue_attrs(wq, ux_wq_attrs_qmi);
		*skip = true;
	} else if (WQ_CMP("oplus_bt_tty")) {
		*ret = __apply_workqueue_attrs(wq, ux_wq_attrs_bt_tty);
		*skip = true;
	}
}

static void android_rvh_alloc_workqueue_handler(void *unused,
						 struct workqueue_struct *wq, unsigned int *flags, int *max_active)
{
	/* qmi_msg_handler and oplus_bt_tty need WQ_UNBOUND | WQ_HIGHPRI flags */
	if (WQ_CMP("qmi_msg_handler") || WQ_CMP("oplus_bt_tty")) {
		if (!wq->unbound_attrs) {
			wq->unbound_attrs = alloc_workqueue_attrs();
			if (!wq->unbound_attrs) {
				pr_err("%s alloc_workqueue_attrs failed: %s", __func__, wq->name);
				return;
			}
		}
		*flags |= (WQ_UNBOUND | WQ_HIGHPRI);
		if (*max_active == 1)
			*flags |= __WQ_ORDERED;
	}
}

static void android_rvh_create_worker_handler(void *unused,
					       struct task_struct *task, struct workqueue_attrs *attrs)
{
	if (attrs->nice == VIRTUAL_KWORKER_QMI_NICE ||
	    attrs->nice == VIRTUAL_KWORKER_BT_TTY_NICE) {
		oplus_set_ux_state_lock(task, SA_TYPE_LIGHT, -1, true);
		if (task->comm[8] == 'u')
			task->comm[8] = 'X';
	}
}

/* ========== Tracepoint Management ========== */

struct tracepoints_table {
	const char *name;
	void *func;
	struct tracepoint *tp;
	bool init;
};

static struct tracepoints_table interests[] = {
	{
		.name = "android_rvh_alloc_and_link_pwqs",
		.func = android_rvh_alloc_and_link_pwqs_handler
	},
	{
		.name = "android_rvh_alloc_workqueue",
		.func = android_rvh_alloc_workqueue_handler
	},
	{
		.name = "android_rvh_create_worker",
		.func = android_rvh_create_worker_handler
	},
};

#define FOR_EACH_INTEREST(i) \
	for (i = 0; i < sizeof(interests) / sizeof(struct tracepoints_table); i++)

static void lookup_tracepoints(struct tracepoint *tp, void *ignore)
{
	int i;

	FOR_EACH_INTEREST(i) {
		if (strcmp(interests[i].name, tp->name) == 0)
			interests[i].tp = tp;
	}
}

static int btuart_install_tracepoints(void)
{
	int i;

	FOR_EACH_INTEREST(i) {
		if (interests[i].tp == NULL) {
			pr_err("%s: tracepoint %s not found\n",
			       THIS_MODULE->name, interests[i].name);
			return -1;
		}

		if (!interests[i].init) {
			tracepoint_probe_register(interests[i].tp,
						  interests[i].func,
						  NULL);
			interests[i].init = true;
		}
	}

	return 0;
}

static void btuart_uninstall_tracepoints(void)
{
	int i;

	FOR_EACH_INTEREST(i) {
		if (interests[i].init) {
			tracepoint_probe_unregister(interests[i].tp,
						    interests[i].func,
						    NULL);
		}
	}
}

/* ========== Kprobe Management ========== */

static int btuart_install_kprobes(void)
{
	int err;
	unsigned long detected_offset;

	pr_info("oplus_btuart_ux: starting kprobe installation...\n");

	/* 1. Detect and register tty_flip_buffer_push (main path, must succeed) */
	pr_info("oplus_btuart_ux: installing kprobe for tty_flip_buffer_push\n");
	detected_offset = detect_kprobe_offset("tty_flip_buffer_push",
					       0x2c,
					       handler_tty_buffer_queue_work);
	if (detected_offset == (unsigned long) -1) {
		pr_err("btuart_ux: tty_flip_buffer_push offset detect fail\n");
		return -EINVAL;
	}

	oplus_tty_flip_buffer_push_kp = kzalloc(sizeof(struct kprobe), GFP_KERNEL);
	if (!oplus_tty_flip_buffer_push_kp) {
		pr_err("btuart_ux: alloc kprobe tty_flip_buffer_push fail\n");
		return -ENOMEM;
	}

	oplus_tty_flip_buffer_push_kp->symbol_name = "tty_flip_buffer_push";
	oplus_tty_flip_buffer_push_kp->offset = detected_offset;
	oplus_tty_flip_buffer_push_kp->pre_handler = handler_tty_buffer_queue_work;

	pr_info("btuart_ux: reg kprobe tty_flip_buffer_push@0x%lx\n",
		detected_offset);
	err = register_kprobe(oplus_tty_flip_buffer_push_kp);
	if (err < 0) {
		pr_err("btuart_ux: reg kprobe tty_flip_buffer_push fail:%d\n",
		       err);
		kfree(oplus_tty_flip_buffer_push_kp);
		oplus_tty_flip_buffer_push_kp = NULL;
		return err;
	}
	kprobe_tty_flip_buffer_push_registered = true;
	pr_info("btuart_ux: tty_flip_buffer_push kprobe ok@0x%lx\n",
		detected_offset);

	/* 2. Register optional kprobes (failures don't affect main func) */
	pr_info("btuart_ux: install kprobe tty_buffer_unlock_exclusive\n");
	detected_offset = detect_kprobe_offset("tty_buffer_unlock_exclusive",
					       0x54,
					       handler_tty_buffer_queue_work);
	if (detected_offset != (unsigned long) -1) {
		oplus_tty_buffer_unlock_exclusive_kp =
			kzalloc(sizeof(struct kprobe), GFP_KERNEL);
		if (oplus_tty_buffer_unlock_exclusive_kp) {
			oplus_tty_buffer_unlock_exclusive_kp->symbol_name =
				"tty_buffer_unlock_exclusive";
			oplus_tty_buffer_unlock_exclusive_kp->offset =
				detected_offset;
			oplus_tty_buffer_unlock_exclusive_kp->pre_handler =
				handler_tty_buffer_queue_work;

			pr_info("btuart_ux: reg kprobe unlock_excl@0x%lx\n",
				detected_offset);
			err = register_kprobe(oplus_tty_buffer_unlock_exclusive_kp);
			if (err == 0) {
				kprobe_tty_buffer_unlock_exclusive_registered =
					true;
				pr_info("btuart_ux: unlock_excl kprobe ok\n");
			} else {
				pr_warn("btuart_ux: unlock_excl fail:%d\n",
					err);
				kfree(oplus_tty_buffer_unlock_exclusive_kp);
				oplus_tty_buffer_unlock_exclusive_kp = NULL;
			}
		}
	} else {
		pr_warn("btuart_ux: unlock_excl offset detect fail\n");
	}

	/* 3. tty_insert_flip_string_and_push_buffer */
	pr_info("btuart_ux: install kprobe insert_flip_string_push\n");
	detected_offset = detect_kprobe_offset(
		"tty_insert_flip_string_and_push_buffer",
		0xec, handler_tty_buffer_queue_work);
	if (detected_offset != (unsigned long) -1) {
		oplus_tty_insert_flip_string_and_push_buffer_kp =
			kzalloc(sizeof(struct kprobe), GFP_KERNEL);
		if (oplus_tty_insert_flip_string_and_push_buffer_kp) {
			oplus_tty_insert_flip_string_and_push_buffer_kp->
				symbol_name =
				"tty_insert_flip_string_and_push_buffer";
			oplus_tty_insert_flip_string_and_push_buffer_kp->
				offset = detected_offset;
			oplus_tty_insert_flip_string_and_push_buffer_kp->
				pre_handler = handler_tty_buffer_queue_work;

			pr_info("btuart_ux: reg kprobe flip_push@0x%lx\n",
				detected_offset);
			err = register_kprobe(
				oplus_tty_insert_flip_string_and_push_buffer_kp);
			if (err == 0) {
				kprobe_tty_insert_flip_string_and_push_buffer_registered = true;
				pr_info("btuart_ux: flip_push kprobe ok\n");
			} else {
				pr_warn("btuart_ux: flip_push fail:%d\n", err);
				kfree(oplus_tty_insert_flip_string_and_push_buffer_kp);
				oplus_tty_insert_flip_string_and_push_buffer_kp =
					NULL;
			}
		}
	} else {
		pr_warn("btuart_ux: flip_push offset detect fail\n");
	}

	/* 4. tty_buffer_restart_work */
	pr_info("btuart_ux: install kprobe tty_buffer_restart_work\n");
	detected_offset = detect_kprobe_offset("tty_buffer_restart_work",
		0x1c, handler_tty_buffer_queue_work);
	if (detected_offset != (unsigned long) -1) {
		oplus_tty_buffer_restart_work_kp =
			kzalloc(sizeof(struct kprobe), GFP_KERNEL);
		if (oplus_tty_buffer_restart_work_kp) {
			oplus_tty_buffer_restart_work_kp->symbol_name =
				"tty_buffer_restart_work";
			oplus_tty_buffer_restart_work_kp->offset =
				detected_offset;
			oplus_tty_buffer_restart_work_kp->pre_handler =
				handler_tty_buffer_queue_work;

			pr_info("btuart_ux: reg kprobe restart@0x%lx\n",
				detected_offset);
			err = register_kprobe(oplus_tty_buffer_restart_work_kp);
			if (err == 0) {
				kprobe_tty_buffer_restart_work_registered = true;
				pr_info("btuart_ux: restart_work kprobe ok\n");
			} else {
				pr_warn("btuart_ux: restart fail:%d\n", err);
				kfree(oplus_tty_buffer_restart_work_kp);
				oplus_tty_buffer_restart_work_kp = NULL;
			}
		}
	} else {
		pr_warn("btuart_ux: restart offset detect fail\n");
	}

	pr_info("btuart_ux: kprobe done. flip=%d,unlock=%d,push=%d,rst=%d\n",
		kprobe_tty_flip_buffer_push_registered,
		kprobe_tty_buffer_unlock_exclusive_registered,
		kprobe_tty_insert_flip_string_and_push_buffer_registered,
		kprobe_tty_buffer_restart_work_registered);

	return 0;
}

static void btuart_uninstall_kprobes(void)
{
	if (kprobe_tty_flip_buffer_push_registered &&
	    oplus_tty_flip_buffer_push_kp) {
		unregister_kprobe(oplus_tty_flip_buffer_push_kp);
		kfree(oplus_tty_flip_buffer_push_kp);
		oplus_tty_flip_buffer_push_kp = NULL;
		kprobe_tty_flip_buffer_push_registered = false;
	}

	if (kprobe_tty_buffer_unlock_exclusive_registered &&
	    oplus_tty_buffer_unlock_exclusive_kp) {
		unregister_kprobe(oplus_tty_buffer_unlock_exclusive_kp);
		kfree(oplus_tty_buffer_unlock_exclusive_kp);
		oplus_tty_buffer_unlock_exclusive_kp = NULL;
		kprobe_tty_buffer_unlock_exclusive_registered = false;
	}

	if (kprobe_tty_insert_flip_string_and_push_buffer_registered &&
	    oplus_tty_insert_flip_string_and_push_buffer_kp) {
		unregister_kprobe(
			oplus_tty_insert_flip_string_and_push_buffer_kp);
		kfree(oplus_tty_insert_flip_string_and_push_buffer_kp);
		oplus_tty_insert_flip_string_and_push_buffer_kp = NULL;
		kprobe_tty_insert_flip_string_and_push_buffer_registered =
			false;
	}

	if (kprobe_tty_buffer_restart_work_registered &&
	    oplus_tty_buffer_restart_work_kp) {
		unregister_kprobe(oplus_tty_buffer_restart_work_kp);
		kfree(oplus_tty_buffer_restart_work_kp);
		oplus_tty_buffer_restart_work_kp = NULL;
		kprobe_tty_buffer_restart_work_registered = false;
	}
}

/* ========== Module Init/Exit ========== */

static int __init oplus_btuart_ux_init(void)
{
	int err = 0;
	struct workqueue_struct *wq;

	pr_info("oplus_btuart_ux: enter\n");

	/* Lookup tracepoints */
	for_each_kernel_tracepoint(lookup_tracepoints, NULL);

	/* Alloc attrs for qmi_msg_handler */
	ux_wq_attrs_qmi = alloc_workqueue_attrs();
	if (!ux_wq_attrs_qmi) {
		pr_err("oplus_btuart_ux: alloc ux_wq_attrs_qmi fail!\n");
		err = -ENOMEM;
		goto out;
	}
	ux_wq_attrs_qmi->nice = VIRTUAL_KWORKER_QMI_NICE;

	/* Alloc attrs for Bluetooth TTY workqueue */
	ux_wq_attrs_bt_tty = alloc_workqueue_attrs();
	if (!ux_wq_attrs_bt_tty) {
		pr_err("oplus_btuart_ux: alloc ux_wq_attrs_bt_tty fail!\n");
		err = -ENOMEM;
		goto err_free_qmi_attrs;
	}
	ux_wq_attrs_bt_tty->nice = VIRTUAL_KWORKER_BT_TTY_NICE;

	/* Install tracepoints first (before creating workqueue) */
	err = btuart_install_tracepoints();
	if (err)
		goto err_free_bt_tty_attrs;

	/* Create workqueue, protected by mutex */
	mutex_lock(&bt_tty_wq_mutex);

	wq = alloc_workqueue("oplus_bt_tty",
			     WQ_UNBOUND | WQ_HIGHPRI | WQ_MEM_RECLAIM,
			     0);
	if (!wq) {
		pr_err("oplus_btuart_ux: failed to create oplus_bt_tty_workqueue\n");
		err = -ENOMEM;
		mutex_unlock(&bt_tty_wq_mutex);
		goto err_uninstall_tracepoints;
	}

	/* Use RCU assignment */
	rcu_assign_pointer(oplus_bt_tty_workqueue, wq);

	mutex_unlock(&bt_tty_wq_mutex);

	/*
	 * Install kprobe on tty buffer functions to redirect Bluetooth TTY
	 * work to our optimized workqueue.
	 */
	err = btuart_install_kprobes();
	if (err)
		goto err_destroy_workqueue;

	/* Mark module as initialized (use memory barrier for visibility) */
	smp_store_release(&module_initialized, true);

	pr_info("oplus_btuart_ux: module loaded successfully\n");
	return 0;

err_destroy_workqueue:
	mutex_lock(&bt_tty_wq_mutex);
	wq = rcu_replace_pointer(oplus_bt_tty_workqueue, NULL,
				 lockdep_is_held(&bt_tty_wq_mutex));
	mutex_unlock(&bt_tty_wq_mutex);
	if (wq) {
		synchronize_rcu();
		destroy_workqueue(wq);
	}
err_uninstall_tracepoints:
	btuart_uninstall_tracepoints();
err_free_bt_tty_attrs:
	free_workqueue_attrs(ux_wq_attrs_bt_tty);
err_free_qmi_attrs:
	free_workqueue_attrs(ux_wq_attrs_qmi);
out:
	return err;
}

static void __exit oplus_btuart_ux_exit(void)
{
	struct workqueue_struct *wq;

	pr_info("oplus_btuart_ux: module unloading...\n");

	/* Step 1: Mark module as shutting down, stop new work submissions */
	spin_lock(&shutdown_lock);
	module_shutting_down = true;
	smp_store_release(&module_initialized, false);
	spin_unlock(&shutdown_lock);

	/* Step 2: Uninstall kprobes first to prevent new work redirection */
	btuart_uninstall_kprobes();

	/* Step 3: Get workqueue pointer and set to NULL */
	mutex_lock(&bt_tty_wq_mutex);
	wq = rcu_replace_pointer(oplus_bt_tty_workqueue, NULL,
				 lockdep_is_held(&bt_tty_wq_mutex));
	mutex_unlock(&bt_tty_wq_mutex);

	if (!wq) {
		pr_warn("oplus_btuart_ux: workqueue already destroyed\n");
		goto cleanup_attrs;
	}

	/* Step 4: Wait for RCU readers to complete */
	synchronize_rcu();

	/* Step 5: Flush workqueue, wait for all pending work to complete */
	pr_info("oplus_btuart_ux: flushing workqueue...\n");
	flush_workqueue(wq);

	/* Step 6: Destroy workqueue */
	destroy_workqueue(wq);
	pr_info("oplus_btuart_ux: workqueue destroyed\n");

cleanup_attrs:
	/* Step 7: Uninstall tracepoints */
	btuart_uninstall_tracepoints();

	/* Step 8: Free workqueue attrs */
	if (ux_wq_attrs_qmi)
		free_workqueue_attrs(ux_wq_attrs_qmi);
	if (ux_wq_attrs_bt_tty)
		free_workqueue_attrs(ux_wq_attrs_bt_tty);

	pr_info("oplus_btuart_ux: module unloaded\n");
}

module_init(oplus_btuart_ux_init);
module_exit(oplus_btuart_ux_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oplus");
MODULE_DESCRIPTION("Workqueue UX priority for btuart");
#endif

