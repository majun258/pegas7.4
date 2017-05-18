/*
 * Copyright (C) 2016 Hisilicon Limited, All Rights Reserved.
 * Author: Zhichang Yuan <yuanzhichang@hisilicon.com>
 * Author: Zou Rongrong <zourongrong@huawei.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/ipmi_smi.h>
#include <linux/kthread.h>

#include "lpc_comm.h"

/* Setting USE_LPC_IPMI_SI_SM to 1 to use LPC defined BT handlers.
 * otherwise will reuse the IPMI BT handles. */
#define USE_LPC_IPMI_SI_SM 0

#ifdef USE_LPC_IPMI_SI_SM
#include "lpc_si_sm.h"
#else
#include "ipmi_si_sm.h"
#endif

/*
 * Setting this bit means each IO operation will target to a
 * different port address:
 * 0 means repeatedly IO operations will stick on the same port,
 * such as BT;
 */
#define FG_INCRADDR_LPC		0x02

/* bounds of the LPC bus address range */
#define LPC_MIN_BUS_RANGE	0x0

/* The maximum continuous operations */
#define LPC_MAX_OPCNT	16
/* only support IO data unit length is four at maximum */
#define LPC_MAX_DULEN	4
#if LPC_MAX_DULEN > LPC_MAX_OPCNT
#error "LPC.. MAX_DULEN must be not bigger than MAX_OPCNT!"
#endif

#define LPC_REG_START		0x00 /* start a new LPC cycle */
#define LPC_REG_OP_STATUS	0x04 /* the current LPC status */
#define LPC_REG_IRQ_ST		0x08 /* interrupt enable&status */
#define LPC_REG_OP_LEN		0x10 /* how many LPC cycles each start */
#define LPC_REG_CMD		0x14 /* command for the required LPC cycle */
#define LPC_REG_ADDR		0x20 /* LPC target address */
#define LPC_REG_WDATA		0x24 /* data to be written */
#define LPC_REG_RDATA		0x28 /* data coming from peer */


/* The command register fields */
#define LPC_CMD_SAMEADDR	0x08
#define LPC_CMD_TYPE_IO		0x00
#define LPC_CMD_WRITE		0x01
#define LPC_CMD_READ		0x00
/* the bit attribute is W1C. 1 represents OK. */
#define LPC_STAT_BYIRQ		0x02

#define LPC_STATUS_IDLE		0x01
#define LPC_OP_FINISHED		0x02

#define START_WORK		0x01

/*
 * The minimal waiting interval... Suggest it is not less than 10.
 * Bigger value probably will lower the performance.
 */
#define LPC_NSEC_PERWAIT	100
/*
 * The maximum waiting time is about 128us.
 * The fastest IO cycle time is about 390ns, but the worst case will wait
 * for extra 256 lpc clocks, so (256 + 13) * 30ns = 8 us. The maximum
 * burst cycles is 16. So, the maximum waiting time is about 128us under
 * worst case.
 * choose 1300 as the maximum.
 */
#define LPC_MAX_WAITCNT		1300
/* About 10us. This is specific for single IO operation, such as inb. */
#define LPC_PEROP_WAITCNT	100

#define LPC_IPMI_ADDR	0xe4
#define LPC_IPMI_IOREG_SIZE 4

struct lpc_cycle_para {
	unsigned int opflags;
	unsigned int csize; /* the data length of each operation */
};

struct hisilpc_dev {
	spinlock_t cycle_lock;
	void __iomem  *membase;
	struct extio_ops *ops;
};

static struct platform_driver hisilpc_driver;

static inline int wait_lpc_idle(unsigned char *mbase,
				unsigned int waitcnt) {
	u32 opstatus;

	while (waitcnt--) {
		ndelay(LPC_NSEC_PERWAIT);
		opstatus = readl(mbase + LPC_REG_OP_STATUS);
		if (opstatus & LPC_STATUS_IDLE)
			return (opstatus & LPC_OP_FINISHED) ? 0 : (-EIO);
	}
	return -ETIME;
}

/*
 * hisilpc_target_in - trigger a series of lpc cycles to read required data
 *		       from target peripheral.
 * @pdev: pointer to hisi lpc device
 * @para: some parameters used to control the lpc I/O operations
 * @ptaddr: the lpc I/O target port address
 * @buf: where the read back data is stored
 * @opcnt: how many I/O operations required in this calling
 *
 * Only one byte data is read each I/O operation.
 *
 * Returns 0 on success, non-zero on fail.
 *
 */
static int
hisilpc_target_in(struct hisilpc_dev *lpcdev, struct lpc_cycle_para *para,
		  unsigned long ptaddr, unsigned char *buf,
		  unsigned long opcnt)
{
	unsigned long cnt_per_trans;
	unsigned int cmd_word;
	unsigned int waitcnt;
	int ret;

	if (!buf || !opcnt || !para || !para->csize || !lpcdev)
		return -EINVAL;

	if (opcnt  > LPC_MAX_OPCNT)
		return -EINVAL;

	cmd_word = LPC_CMD_TYPE_IO | LPC_CMD_READ;
	waitcnt = LPC_PEROP_WAITCNT;
	if (!(para->opflags & FG_INCRADDR_LPC)) {
		cmd_word |= LPC_CMD_SAMEADDR;
		waitcnt = LPC_MAX_WAITCNT;
	}

	ret = 0;
	cnt_per_trans = (para->csize == 1) ? opcnt : para->csize;
	for (; opcnt && !ret; cnt_per_trans = para->csize) {
		unsigned long flags;

		/* whole operation must be atomic */
		spin_lock_irqsave(&lpcdev->cycle_lock, flags);

		writel(cnt_per_trans, lpcdev->membase + LPC_REG_OP_LEN);

		writel(cmd_word, lpcdev->membase + LPC_REG_CMD);

		writel(ptaddr, lpcdev->membase + LPC_REG_ADDR);

		writel(START_WORK, lpcdev->membase + LPC_REG_START);

		/* whether the operation is finished */
		ret = wait_lpc_idle(lpcdev->membase, waitcnt);
		if (!ret) {
			opcnt -= cnt_per_trans;
			for (; cnt_per_trans--; buf++)
				*buf = readl(lpcdev->membase + LPC_REG_RDATA);
		}

		spin_unlock_irqrestore(&lpcdev->cycle_lock, flags);
	}

	return ret;
}

/**
 * hisilpc_target_out - trigger a series of lpc cycles to write required data
 *		  to target peripheral.
 * @pdev: pointer to hisi lpc device
 * @para: some parameters used to control the lpc I/O operations
 * @ptaddr: the lpc I/O target port address
 * @buf: where the data to be written is stored
 * @opcnt: how many I/O operations required
 *
 * Only one byte data is read each I/O operation.
 *
 * Returns 0 on success, non-zero on fail.
 *
 */
static int
hisilpc_target_out(struct hisilpc_dev *lpcdev, struct lpc_cycle_para *para,
		   unsigned long ptaddr, const unsigned char *buf,
		   unsigned long opcnt)
{
	unsigned long cnt_per_trans;
	unsigned int cmd_word;
	unsigned int waitcnt;
	int ret;

	if (!buf || !opcnt || !para || !lpcdev)
		return -EINVAL;

	if (opcnt > LPC_MAX_OPCNT)
		return -EINVAL;
	/* default is increasing address */
	cmd_word = LPC_CMD_TYPE_IO | LPC_CMD_WRITE;
	waitcnt = LPC_PEROP_WAITCNT;
	if (!(para->opflags & FG_INCRADDR_LPC)) {
		cmd_word |= LPC_CMD_SAMEADDR;
		waitcnt = LPC_MAX_WAITCNT;
	}

	ret = 0;
	cnt_per_trans = (para->csize == 1) ? opcnt : para->csize;
	for (; opcnt && !ret; cnt_per_trans = para->csize) {
		unsigned long flags;

		spin_lock_irqsave(&lpcdev->cycle_lock, flags);

		writel(cnt_per_trans, lpcdev->membase + LPC_REG_OP_LEN);
		opcnt -= cnt_per_trans;
		for (; cnt_per_trans--; buf++)
			writel(*buf, lpcdev->membase + LPC_REG_WDATA);

		writel(cmd_word, lpcdev->membase + LPC_REG_CMD);

		writel(ptaddr, lpcdev->membase + LPC_REG_ADDR);

		writel(START_WORK, lpcdev->membase + LPC_REG_START);

		/* whether the operation is finished */
		ret = wait_lpc_idle(lpcdev->membase, waitcnt);

		spin_unlock_irqrestore(&lpcdev->cycle_lock, flags);
	}

	return ret;
}

/* convert the bus-local PIO to linux virtual PIO */
static inline unsigned long
hisi_lpc_pio_to_addr(struct hisilpc_dev *lpcdev, unsigned long pio)
{
	return pio;
}


/**
 * hisilpc_comm_in - read/input the data from the I/O peripheral through LPC.
 * @devobj: pointer to the device information relevant to LPC controller.
 * @pio: the target I/O port address.
 * @dlen: the data length required to read from the target I/O port.
 *
 * when succeed, the data read back is stored in buffer pointed by inbuf.
 * For inb, return the data read from I/O or -1 when error occur.
 */
static u64 hisilpc_comm_in(void *devobj, unsigned long pio, size_t dlen)
{
	struct hisilpc_dev *lpcdev;
	struct lpc_cycle_para iopara;
	u32 rd_data;
	unsigned char *newbuf;
	int ret = 0;
	unsigned long ptaddr;

	if (!devobj || !dlen || dlen > LPC_MAX_DULEN ||	(dlen & (dlen - 1)))
		return -1;

	/* the local buffer must be enough for one data unit */
	if (sizeof(rd_data) < dlen)
		return -1;

	newbuf = (unsigned char *)&rd_data;

	lpcdev = (struct hisilpc_dev *)devobj;

	ptaddr = hisi_lpc_pio_to_addr(lpcdev, pio);

	iopara.opflags = FG_INCRADDR_LPC;
	iopara.csize = dlen;

	ret = hisilpc_target_in(lpcdev, &iopara, ptaddr, newbuf, dlen);
	if (ret)
		return -1;

	return le32_to_cpu(rd_data);
}

/**
 * hisilpc_comm_out - output the data whose maximum length is four bytes
		      to the I/O peripheral through the LPC host.
 * @devobj: pointer to the device information relevant to LPC controller.
 * @outval: a value to be outputted from caller, maximum is four bytes.
 * @pio: the target I/O port address.
 * @dlen: the data length required writing to the target I/O port.
 *
 * This function is corresponding to out(b,w,l) only
 *
 */
static void hisilpc_comm_out(void *devobj, unsigned long pio,
			     u32 outval, size_t dlen)
{
	struct hisilpc_dev *lpcdev;
	struct lpc_cycle_para iopara;
	const unsigned char *newbuf;
	unsigned long ptaddr;

	if (!devobj || !dlen || dlen > LPC_MAX_DULEN)
		return;

	if (sizeof(outval) < dlen)
		return;

	outval = cpu_to_le32(outval);

	newbuf = (const unsigned char *)&outval;
	lpcdev = (struct hisilpc_dev *)devobj;

	ptaddr = hisi_lpc_pio_to_addr(lpcdev, pio);

	iopara.opflags = FG_INCRADDR_LPC;
	iopara.csize = dlen;

	hisilpc_target_out(lpcdev, &iopara, ptaddr, newbuf, dlen);
}

/*
 * hisilpc_comm_ins - read/input the data in buffer to the I/O peripheral
 *		    through LPC, it corresponds to ins(b,w,l)
 * @devobj: pointer to the device information relevant to LPC controller.
 * @pio: the target I/O port address.
 * @inbuf: a buffer where read/input data bytes are stored.
 * @dlen: the data length required writing to the target I/O port.
 * @count: how many data units whose length is dlen will be read.
 *
 */
static u64
hisilpc_comm_ins(void *devobj, unsigned long pio, void *inbuf,
		 size_t dlen, unsigned int count)
{
	struct hisilpc_dev *lpcdev;
	struct lpc_cycle_para iopara;
	unsigned char *newbuf;
	unsigned int loopcnt, cntleft;
	unsigned int max_perburst;
	unsigned long ptaddr;

	if (!devobj || !inbuf || !count || !dlen ||
			dlen > LPC_MAX_DULEN || (dlen & (dlen - 1)))
		return -1;

	iopara.opflags = 0;
	if (dlen > 1)
		iopara.opflags |= FG_INCRADDR_LPC;
	iopara.csize = dlen;

	lpcdev = (struct hisilpc_dev *)devobj;
	ptaddr = hisi_lpc_pio_to_addr(lpcdev, pio);
	newbuf = (unsigned char *)inbuf;
	/*
	 * ensure data stream whose length is multiple of dlen to be processed
	 * each IO input
	 */
	max_perburst = LPC_MAX_OPCNT & (~(dlen - 1));
	cntleft = count * dlen;
	do {
		int ret;

		loopcnt = (cntleft >= max_perburst) ? max_perburst : cntleft;
		ret = hisilpc_target_in(lpcdev, &iopara, ptaddr,
					newbuf, loopcnt);
		if (ret)
			return ret;
		newbuf += loopcnt;
		cntleft -= loopcnt;
	} while (cntleft);

	return 0;
}

/*
 * hisilpc_comm_outs - write/output the data in buffer to the I/O peripheral
 *		    through LPC, it corresponds to outs(b,w,l)
 * @devobj: pointer to the device information relevant to LPC controller.
 * @pio: the target I/O port address.
 * @outbuf: a buffer where write/output data bytes are stored.
 * @dlen: the data length required writing to the target I/O port .
 * @count: how many data units whose length is dlen will be written.
 *
 */
static void
hisilpc_comm_outs(void *devobj, unsigned long pio, const void *outbuf,
		  size_t dlen, unsigned int count)
{
	struct hisilpc_dev *lpcdev;
	struct lpc_cycle_para iopara;
	const unsigned char *newbuf;
	unsigned int loopcnt, cntleft;
	unsigned int max_perburst;
	unsigned long ptaddr;
	int ret = 0;

	if (!devobj || !outbuf || !count || !dlen ||
			dlen > LPC_MAX_DULEN || (dlen & (dlen - 1)))
		return;

	iopara.opflags = 0;
	if (dlen > 1)
		iopara.opflags |= FG_INCRADDR_LPC;
	iopara.csize = dlen;

	lpcdev = (struct hisilpc_dev *)devobj;
	ptaddr = hisi_lpc_pio_to_addr(lpcdev, pio);
	newbuf = (unsigned char *)outbuf;
	/*
	 * ensure data stream whose lenght is multiple of dlen to be processed
	 * each IO input
	 */
	max_perburst = LPC_MAX_OPCNT & (~(dlen - 1));
	cntleft = count * dlen;
	do {
		loopcnt = (cntleft >= max_perburst) ? max_perburst : cntleft;
		ret = hisilpc_target_out(lpcdev, &iopara, ptaddr, newbuf,
						loopcnt);
		if (ret)
			break;
		newbuf += loopcnt;
		cntleft -= loopcnt;
	} while (cntleft);
}

static struct extio_ops hisi_lpc_ops = {
	.pfin = hisilpc_comm_in,
	.pfout = hisilpc_comm_out,
	.pfins = hisilpc_comm_ins,
	.pfouts = hisilpc_comm_outs,
};

/* Begin of ipmi processing. */
#define PFX "hisi_ipmi_si: "

/* Call every 10 ms. */
#define SI_TIMEOUT_TIME_USEC	10000
#define SI_USEC_PER_JIFFY	(1000000/HZ)
#define SI_TIMEOUT_JIFFIES	(SI_TIMEOUT_TIME_USEC/SI_USEC_PER_JIFFY)
#define SI_SHORT_TIMEOUT_USEC  250 /* .25ms when the SM request a
				      short timeout */

enum si_intf_state {
	SI_NORMAL,
	SI_GETTING_FLAGS,
	SI_GETTING_EVENTS,
	SI_CLEARING_FLAGS,
	SI_GETTING_MESSAGES,
	SI_CHECKING_ENABLES,
	SI_SETTING_ENABLES
	/* FIXME - add watchdog stuff. */
};

/* Some BT-specific defines we need here. */
#define IPMI_BT_INTMASK_REG		2
#define IPMI_BT_INTMASK_CLEAR_IRQ_BIT	2
#define IPMI_BT_INTMASK_ENABLE_IRQ_BIT	1

enum si_type {
	SI_KCS, SI_SMIC, SI_BT
};

struct lpc_dev_data {
	/* the start bus-local I/O address */
	unsigned long start_adr;
	unsigned long iosize;
	struct extio_ops *hostops;
};

static const char * const si_to_str[] = { "kcs", "smic", "bt" };

#define DEVICE_NAME "hisi_ipmi_si"

/* Measure times between events in the driver. */
#undef DEBUG_TIMING

static struct platform_driver ipmi_driver;

/*
 * Indexes into stats[] in smi_info below.
 */
enum si_stat_indexes {
	/*
	 * Number of times the driver requested a timer while an operation
	 * was in progress.
	 */
	SI_STAT_short_timeouts = 0,

	/*
	 * Number of times the driver requested a timer while nothing was in
	 * progress.
	 */
	SI_STAT_long_timeouts,

	/* Number of times the interface was idle while being polled. */
	SI_STAT_idles,

	/* Number of interrupts the driver handled. */
	SI_STAT_interrupts,

	/* Number of time the driver got an ATTN from the hardware. */
	SI_STAT_attentions,

	/* Number of times the driver requested flags from the hardware. */
	SI_STAT_flag_fetches,

	/* Number of times the hardware didn't follow the state machine. */
	SI_STAT_hosed_count,

	/* Number of completed messages. */
	SI_STAT_complete_transactions,

	/* Number of IPMI events received from the hardware. */
	SI_STAT_events,

	/* Number of watchdog pretimeouts. */
	SI_STAT_watchdog_pretimeouts,

	/* Number of asynchronous messages received. */
	SI_STAT_incoming_messages,


	/* This *must* remain last, add new values above this. */
	SI_NUM_STATS
};

struct smi_info {
	int                    intf_num;
	ipmi_smi_t             intf;
	struct si_sm_data      *si_sm;
	const struct si_sm_handlers *handlers;
	enum si_type           si_type;
	spinlock_t             si_lock;
	struct ipmi_smi_msg    *waiting_msg;
	struct ipmi_smi_msg    *curr_msg;
	enum si_intf_state     si_state;

	/*
	 * Used to handle the various types of I/O that can occur with
	 * IPMI
	 */
	struct si_sm_io io;
	int (*io_setup)(struct smi_info *info);
	void (*io_cleanup)(struct smi_info *info);
	int (*irq_setup)(struct smi_info *info);
	void (*irq_cleanup)(struct smi_info *info);
	unsigned int io_size;
	enum ipmi_addr_src addr_source; /* ACPI, PCI, SMBIOS, hardcode, etc. */
	void (*addr_source_cleanup)(struct smi_info *info);
	void *addr_source_data;

	/*
	 * Per-OEM handler, called from handle_flags().  Returns 1
	 * when handle_flags() needs to be re-run or 0 indicating it
	 * set si_state itself.
	 */
	int (*oem_data_avail_handler)(struct smi_info *smi_info);

	/*
	 * Flags from the last GET_MSG_FLAGS command, used when an ATTN
	 * is set to hold the flags until we are done handling everything
	 * from the flags.
	 */
#define RECEIVE_MSG_AVAIL	0x01
#define EVENT_MSG_BUFFER_FULL	0x02
#define WDT_PRE_TIMEOUT_INT	0x08
#define OEM0_DATA_AVAIL     0x20
#define OEM1_DATA_AVAIL     0x40
#define OEM2_DATA_AVAIL     0x80
#define OEM_DATA_AVAIL      (OEM0_DATA_AVAIL | \
			     OEM1_DATA_AVAIL | \
			     OEM2_DATA_AVAIL)
	unsigned char       msg_flags;

	/* Does the BMC have an event buffer? */
	bool		    has_event_buffer;

	/*
	 * If set to true, this will request events the next time the
	 * state machine is idle.
	 */
	atomic_t            req_events;

	/*
	 * If true, run the state machine to completion on every send
	 * call.  Generally used after a panic to make sure stuff goes
	 * out.
	 */
	bool                run_to_completion;

	/* The I/O port of an SI interface. */
	int                 port;

	/*
	 * The space between start addresses of the two ports.  For
	 * instance, if the first port is 0xca2 and the spacing is 4, then
	 * the second port is 0xca6.
	 */
	unsigned int        spacing;

	/* zero if no irq; */
	int                 irq;

	/* The timer for this si. */
	struct timer_list   si_timer;

	/* This flag is set, if the timer is running (timer_pending() isn't enough) */
	bool		    timer_running;

	/* The time (in jiffies) the last timeout occurred at. */
	unsigned long       last_timeout_jiffies;

	/* Are we waiting for the events, pretimeouts, received msgs? */
	atomic_t            need_watch;

	/*
	 * The driver will disable interrupts when it gets into a
	 * situation where it cannot handle messages due to lack of
	 * memory.  Once that situation clears up, it will re-enable
	 * interrupts.
	 */
	bool interrupt_disabled;

	/*
	 * Does the BMC support events?
	 */
	bool supports_event_msg_buff;

	/*
	 * Can we disable interrupts the global enables receive irq
	 * bit?  There are currently two forms of brokenness, some
	 * systems cannot disable the bit (which is technically within
	 * the spec but a bad idea) and some systems have the bit
	 * forced to zero even though interrupts work (which is
	 * clearly outside the spec).  The next bool tells which form
	 * of brokenness is present.
	 */
	bool cannot_disable_irq;

	/*
	 * Some systems are broken and cannot set the irq enable
	 * bit, even if they support interrupts.
	 */
	bool irq_enable_broken;

	/*
	 * Did we get an attention that we did not handle?
	 */
	bool got_attn;

	/* From the get device id response... */
	struct ipmi_device_id device_id;

	/* Driver model stuff. */
	struct device *dev;
	struct platform_device *pdev;

	/*
	 * True if we allocated the device, false if it came from
	 * someplace else (like PCI).
	 */
	bool dev_registered;

	/* Slave address, could be reported from DMI. */
	unsigned char slave_addr;

	/* Counters and things for the proc filesystem. */
	atomic_t stats[SI_NUM_STATS];

	struct task_struct *thread;

	struct list_head link;
	union ipmi_smi_info_union addr_info;

	struct extio_ops *ops;
};
#define to_smi_info(d) container_of(d, struct smi_info, io)

#define smi_inc_stat(smi, stat) \
	atomic_inc(&(smi)->stats[SI_STAT_ ## stat])
#define smi_get_stat(smi, stat) \
	((unsigned int) atomic_read(&(smi)->stats[SI_STAT_ ## stat]))

#define SI_MAX_PARMS 4

static int force_kipmid[SI_MAX_PARMS];
static int num_force_kipmid;

static unsigned int kipmid_max_busy_us[SI_MAX_PARMS];
static int num_max_busy_us;

static int try_smi_init(struct smi_info *smi);
static void cleanup_one_si(struct smi_info *to_clean);

#ifdef DEBUG_TIMING
void debug_timestamp(char *msg)
{
	struct timespec64 t;

	getnstimeofday64(&t);
	pr_debug("**%s: %lld.%9.9ld\n", msg, (long long) t.tv_sec, t.tv_nsec);
}
#else
#define debug_timestamp(x)
#endif

static void deliver_recv_msg(struct smi_info *smi_info,
			     struct ipmi_smi_msg *msg)
{
	/* Deliver the message to the upper layer. */
	if (smi_info->intf)
		ipmi_smi_msg_received(smi_info->intf, msg);
	else
		ipmi_free_smi_msg(msg);
}

static void return_hosed_msg(struct smi_info *smi_info, int cCode)
{
	struct ipmi_smi_msg *msg = smi_info->curr_msg;

	if (cCode < 0 || cCode > IPMI_ERR_UNSPECIFIED)
		cCode = IPMI_ERR_UNSPECIFIED;
	/* else use it as is */

	/* Make it a response */
	msg->rsp[0] = msg->data[0] | 4;
	msg->rsp[1] = msg->data[1];
	msg->rsp[2] = cCode;
	msg->rsp_size = 3;

	smi_info->curr_msg = NULL;
	deliver_recv_msg(smi_info, msg);
}

static enum si_sm_result start_next_msg(struct smi_info *smi_info)
{
	int              rv;

	if (!smi_info->waiting_msg) {
		smi_info->curr_msg = NULL;
		rv = SI_SM_IDLE;
	} else {
		int err;

		smi_info->curr_msg = smi_info->waiting_msg;
		smi_info->waiting_msg = NULL;
		debug_timestamp("Start2");
		err = smi_info->handlers->start_transaction(
			smi_info->si_sm,
			smi_info->curr_msg->data,
			smi_info->curr_msg->data_size);
		if (err)
			return_hosed_msg(smi_info, err);

		rv = SI_SM_CALL_WITHOUT_DELAY;
	}

	return rv;
}

static void smi_mod_timer(struct smi_info *smi_info, unsigned long new_val)
{
	smi_info->last_timeout_jiffies = jiffies;
	mod_timer(&smi_info->si_timer, new_val);
	smi_info->timer_running = true;
}

/*
 * Start a new message and (re)start the timer and thread.
 */
static void start_new_msg(struct smi_info *smi_info, unsigned char *msg,
			  unsigned int size)
{
	smi_mod_timer(smi_info, jiffies + SI_TIMEOUT_JIFFIES);

	if (smi_info->thread)
		wake_up_process(smi_info->thread);

	smi_info->handlers->start_transaction(smi_info->si_sm, msg, size);
}

static void start_check_enables(struct smi_info *smi_info, bool start_timer)
{
	unsigned char msg[2];

	msg[0] = (IPMI_NETFN_APP_REQUEST << 2);
	msg[1] = IPMI_GET_BMC_GLOBAL_ENABLES_CMD;

	if (start_timer)
		start_new_msg(smi_info, msg, 2);
	else
		smi_info->handlers->start_transaction(smi_info->si_sm, msg, 2);
	smi_info->si_state = SI_CHECKING_ENABLES;
}

static void start_clear_flags(struct smi_info *smi_info, bool start_timer)
{
	unsigned char msg[3];

	/* Make sure the watchdog pre-timeout flag is not set at startup. */
	msg[0] = (IPMI_NETFN_APP_REQUEST << 2);
	msg[1] = IPMI_CLEAR_MSG_FLAGS_CMD;
	msg[2] = WDT_PRE_TIMEOUT_INT;

	if (start_timer)
		start_new_msg(smi_info, msg, 3);
	else
		smi_info->handlers->start_transaction(smi_info->si_sm, msg, 3);
	smi_info->si_state = SI_CLEARING_FLAGS;
}

static void start_getting_msg_queue(struct smi_info *smi_info)
{
	smi_info->curr_msg->data[0] = (IPMI_NETFN_APP_REQUEST << 2);
	smi_info->curr_msg->data[1] = IPMI_GET_MSG_CMD;
	smi_info->curr_msg->data_size = 2;

	start_new_msg(smi_info, smi_info->curr_msg->data,
		      smi_info->curr_msg->data_size);
	smi_info->si_state = SI_GETTING_MESSAGES;
}

static void start_getting_events(struct smi_info *smi_info)
{
	smi_info->curr_msg->data[0] = (IPMI_NETFN_APP_REQUEST << 2);
	smi_info->curr_msg->data[1] = IPMI_READ_EVENT_MSG_BUFFER_CMD;
	smi_info->curr_msg->data_size = 2;

	start_new_msg(smi_info, smi_info->curr_msg->data,
		      smi_info->curr_msg->data_size);
	smi_info->si_state = SI_GETTING_EVENTS;
}

/*
 * When we have a situtaion where we run out of memory and cannot
 * allocate messages, we just leave them in the BMC and run the system
 * polled until we can allocate some memory.  Once we have some
 * memory, we will re-enable the interrupt.
 *
 * Note that we cannot just use disable_irq(), since the interrupt may
 * be shared.
 */
static inline bool disable_si_irq(struct smi_info *smi_info, bool start_timer)
{
	if ((smi_info->irq) && (!smi_info->interrupt_disabled)) {
		smi_info->interrupt_disabled = true;
		start_check_enables(smi_info, start_timer);
		return true;
	}
	return false;
}

static inline bool enable_si_irq(struct smi_info *smi_info)
{
	if ((smi_info->irq) && (smi_info->interrupt_disabled)) {
		smi_info->interrupt_disabled = false;
		start_check_enables(smi_info, true);
		return true;
	}
	return false;
}

/*
 * Allocate a message.  If unable to allocate, start the interrupt
 * disable process and return NULL.  If able to allocate but
 * interrupts are disabled, free the message and return NULL after
 * starting the interrupt enable process.
 */
static struct ipmi_smi_msg *alloc_msg_handle_irq(struct smi_info *smi_info)
{
	struct ipmi_smi_msg *msg;

	msg = ipmi_alloc_smi_msg();
	if (!msg) {
		if (!disable_si_irq(smi_info, true))
			smi_info->si_state = SI_NORMAL;
	} else if (enable_si_irq(smi_info)) {
		ipmi_free_smi_msg(msg);
		msg = NULL;
	}
	return msg;
}

static void handle_flags(struct smi_info *smi_info)
{
retry:
	if (smi_info->msg_flags & WDT_PRE_TIMEOUT_INT) {
		/* Watchdog pre-timeout */
		smi_inc_stat(smi_info, watchdog_pretimeouts);

		start_clear_flags(smi_info, true);
		smi_info->msg_flags &= ~WDT_PRE_TIMEOUT_INT;
		if (smi_info->intf)
			ipmi_smi_watchdog_pretimeout(smi_info->intf);
	} else if (smi_info->msg_flags & RECEIVE_MSG_AVAIL) {
		/* Messages available. */
		smi_info->curr_msg = alloc_msg_handle_irq(smi_info);
		if (!smi_info->curr_msg)
			return;

		start_getting_msg_queue(smi_info);
	} else if (smi_info->msg_flags & EVENT_MSG_BUFFER_FULL) {
		/* Events available. */
		smi_info->curr_msg = alloc_msg_handle_irq(smi_info);
		if (!smi_info->curr_msg)
			return;

		start_getting_events(smi_info);
	} else if (smi_info->msg_flags & OEM_DATA_AVAIL &&
		   smi_info->oem_data_avail_handler) {
		if (smi_info->oem_data_avail_handler(smi_info))
			goto retry;
	} else
		smi_info->si_state = SI_NORMAL;
}

/*
 * Global enables we care about.
 */
#define GLOBAL_ENABLES_MASK (IPMI_BMC_EVT_MSG_BUFF | IPMI_BMC_RCV_MSG_INTR | \
			     IPMI_BMC_EVT_MSG_INTR)

static u8 current_global_enables(struct smi_info *smi_info, u8 base,
				 bool *irq_on)
{
	u8 enables = 0;

	if (smi_info->supports_event_msg_buff)
		enables |= IPMI_BMC_EVT_MSG_BUFF;

	if (((smi_info->irq && !smi_info->interrupt_disabled) ||
	     smi_info->cannot_disable_irq) &&
	    !smi_info->irq_enable_broken)
		enables |= IPMI_BMC_RCV_MSG_INTR;

	if (smi_info->supports_event_msg_buff &&
	    smi_info->irq && !smi_info->interrupt_disabled &&
	    !smi_info->irq_enable_broken)
		enables |= IPMI_BMC_EVT_MSG_INTR;

	*irq_on = enables & (IPMI_BMC_EVT_MSG_INTR | IPMI_BMC_RCV_MSG_INTR);

	return enables;
}

static void check_bt_irq(struct smi_info *smi_info, bool irq_on)
{
	u8 irqstate = smi_info->io.inputb(&smi_info->io, IPMI_BT_INTMASK_REG);

	irqstate &= IPMI_BT_INTMASK_ENABLE_IRQ_BIT;

	if ((bool)irqstate == irq_on)
		return;

	if (irq_on)
		smi_info->io.outputb(&smi_info->io, IPMI_BT_INTMASK_REG,
				     IPMI_BT_INTMASK_ENABLE_IRQ_BIT);
	else
		smi_info->io.outputb(&smi_info->io, IPMI_BT_INTMASK_REG, 0);
}

static void handle_transaction_done(struct smi_info *smi_info)
{
	struct ipmi_smi_msg *msg;

	debug_timestamp("Done");
	switch (smi_info->si_state) {
	case SI_NORMAL:
		if (!smi_info->curr_msg)
			break;

		smi_info->curr_msg->rsp_size
			= smi_info->handlers->get_result(
				smi_info->si_sm,
				smi_info->curr_msg->rsp,
				IPMI_MAX_MSG_LENGTH);

		/*
		 * Do this here becase deliver_recv_msg() releases the
		 * lock, and a new message can be put in during the
		 * time the lock is released.
		 */
		msg = smi_info->curr_msg;
		smi_info->curr_msg = NULL;
		deliver_recv_msg(smi_info, msg);
		break;

	case SI_GETTING_FLAGS:
	{
		unsigned char msg[4];
		unsigned int  len;

		/* We got the flags from the SMI, now handle them. */
		len = smi_info->handlers->get_result(smi_info->si_sm, msg, 4);
		if (msg[2] != 0) {
			/* Error fetching flags, just give up for now. */
			smi_info->si_state = SI_NORMAL;
		} else if (len < 4) {
			/*
			 * Hmm, no flags.  That's technically illegal, but
			 * don't use uninitialized data.
			 */
			smi_info->si_state = SI_NORMAL;
		} else {
			smi_info->msg_flags = msg[3];
			handle_flags(smi_info);
		}
		break;
	}

	case SI_CLEARING_FLAGS:
	{
		unsigned char msg[3];

		/* We cleared the flags. */
		smi_info->handlers->get_result(smi_info->si_sm, msg, 3);
		if (msg[2] != 0) {
			/* Error clearing flags */
			dev_warn(smi_info->dev,
				 "Error clearing flags: %2.2x\n", msg[2]);
		}
		smi_info->si_state = SI_NORMAL;
		break;
	}

	case SI_GETTING_EVENTS:
	{
		smi_info->curr_msg->rsp_size
			= smi_info->handlers->get_result(
				smi_info->si_sm,
				smi_info->curr_msg->rsp,
				IPMI_MAX_MSG_LENGTH);

		/*
		 * Do this here becase deliver_recv_msg() releases the
		 * lock, and a new message can be put in during the
		 * time the lock is released.
		 */
		msg = smi_info->curr_msg;
		smi_info->curr_msg = NULL;
		if (msg->rsp[2] != 0) {
			/* Error getting event, probably done. */
			msg->done(msg);

			/* Take off the event flag. */
			smi_info->msg_flags &= ~EVENT_MSG_BUFFER_FULL;
			handle_flags(smi_info);
		} else {
			smi_inc_stat(smi_info, events);

			/*
			 * Do this before we deliver the message
			 * because delivering the message releases the
			 * lock and something else can mess with the
			 * state.
			 */
			handle_flags(smi_info);

			deliver_recv_msg(smi_info, msg);
		}
		break;
	}

	case SI_GETTING_MESSAGES:
	{
		smi_info->curr_msg->rsp_size
			= smi_info->handlers->get_result(
				smi_info->si_sm,
				smi_info->curr_msg->rsp,
				IPMI_MAX_MSG_LENGTH);

		/*
		 * Do this here becase deliver_recv_msg() releases the
		 * lock, and a new message can be put in during the
		 * time the lock is released.
		 */
		msg = smi_info->curr_msg;
		smi_info->curr_msg = NULL;
		if (msg->rsp[2] != 0) {
			/* Error getting event, probably done. */
			msg->done(msg);

			/* Take off the msg flag. */
			smi_info->msg_flags &= ~RECEIVE_MSG_AVAIL;
			handle_flags(smi_info);
		} else {
			smi_inc_stat(smi_info, incoming_messages);

			/*
			 * Do this before we deliver the message
			 * because delivering the message releases the
			 * lock and something else can mess with the
			 * state.
			 */
			handle_flags(smi_info);

			deliver_recv_msg(smi_info, msg);
		}
		break;
	}

	case SI_CHECKING_ENABLES:
	{
		unsigned char msg[4];
		u8 enables;
		bool irq_on;

		/* We got the flags from the SMI, now handle them. */
		smi_info->handlers->get_result(smi_info->si_sm, msg, 4);
		if (msg[2] != 0) {
			dev_warn(smi_info->dev,
				 "Couldn't get irq info: %x.\n", msg[2]);
			dev_warn(smi_info->dev,
				 "Maybe ok, but ipmi might run very slowly.\n");
			smi_info->si_state = SI_NORMAL;
			break;
		}
		enables = current_global_enables(smi_info, 0, &irq_on);
		if (smi_info->si_type == SI_BT)
			/* BT has its own interrupt enable bit. */
			check_bt_irq(smi_info, irq_on);
		if (enables != (msg[3] & GLOBAL_ENABLES_MASK)) {
			/* Enables are not correct, fix them. */
			msg[0] = (IPMI_NETFN_APP_REQUEST << 2);
			msg[1] = IPMI_SET_BMC_GLOBAL_ENABLES_CMD;
			msg[2] = enables | (msg[3] & ~GLOBAL_ENABLES_MASK);
			smi_info->handlers->start_transaction(
				smi_info->si_sm, msg, 3);
			smi_info->si_state = SI_SETTING_ENABLES;
		} else if (smi_info->supports_event_msg_buff) {
			smi_info->curr_msg = ipmi_alloc_smi_msg();
			if (!smi_info->curr_msg) {
				smi_info->si_state = SI_NORMAL;
				break;
			}
			start_getting_msg_queue(smi_info);
		} else {
			smi_info->si_state = SI_NORMAL;
		}
		break;
	}

	case SI_SETTING_ENABLES:
	{
		unsigned char msg[4];

		smi_info->handlers->get_result(smi_info->si_sm, msg, 4);
		if (msg[2] != 0)
			dev_warn(smi_info->dev,
				 "Could not set the global enables: 0x%x.\n",
				 msg[2]);

		if (smi_info->supports_event_msg_buff) {
			smi_info->curr_msg = ipmi_alloc_smi_msg();
			if (!smi_info->curr_msg) {
				smi_info->si_state = SI_NORMAL;
				break;
			}
			start_getting_msg_queue(smi_info);
		} else {
			smi_info->si_state = SI_NORMAL;
		}
		break;
	}
	}
}

/*
 * Called on timeouts and events.  Timeouts should pass the elapsed
 * time, interrupts should pass in zero.  Must be called with
 * si_lock held and interrupts disabled.
 */
static enum si_sm_result smi_event_handler(struct smi_info *smi_info,
					   int time)
{
	enum si_sm_result si_sm_result;

restart:
	/*
	 * There used to be a loop here that waited a little while
	 * (around 25us) before giving up.  That turned out to be
	 * pointless, the minimum delays I was seeing were in the 300us
	 * range, which is far too long to wait in an interrupt.  So
	 * we just run until the state machine tells us something
	 * happened or it needs a delay.
	 */
	si_sm_result = smi_info->handlers->event(smi_info->si_sm, time);
	time = 0;
	while (si_sm_result == SI_SM_CALL_WITHOUT_DELAY)
		si_sm_result = smi_info->handlers->event(smi_info->si_sm, 0);

	if (si_sm_result == SI_SM_TRANSACTION_COMPLETE) {
		smi_inc_stat(smi_info, complete_transactions);

		handle_transaction_done(smi_info);
		goto restart;
	} else if (si_sm_result == SI_SM_HOSED) {
		smi_inc_stat(smi_info, hosed_count);

		/*
		 * Do the before return_hosed_msg, because that
		 * releases the lock.
		 */
		smi_info->si_state = SI_NORMAL;
		if (smi_info->curr_msg != NULL) {
			/*
			 * If we were handling a user message, format
			 * a response to send to the upper layer to
			 * tell it about the error.
			 */
			return_hosed_msg(smi_info, IPMI_ERR_UNSPECIFIED);
		}
		goto restart;
	}

	/*
	 * We prefer handling attn over new messages.  But don't do
	 * this if there is not yet an upper layer to handle anything.
	 */
	if (likely(smi_info->intf) &&
	    (si_sm_result == SI_SM_ATTN || smi_info->got_attn)) {
		unsigned char msg[2];

		if (smi_info->si_state != SI_NORMAL) {
			/*
			 * We got an ATTN, but we are doing something else.
			 * Handle the ATTN later.
			 */
			smi_info->got_attn = true;
		} else {
			smi_info->got_attn = false;
			smi_inc_stat(smi_info, attentions);

			/*
			 * Got a attn, send down a get message flags to see
			 * what's causing it.  It would be better to handle
			 * this in the upper layer, but due to the way
			 * interrupts work with the SMI, that's not really
			 * possible.
			 */
			msg[0] = (IPMI_NETFN_APP_REQUEST << 2);
			msg[1] = IPMI_GET_MSG_FLAGS_CMD;

			start_new_msg(smi_info, msg, 2);
			smi_info->si_state = SI_GETTING_FLAGS;
			goto restart;
		}
	}

	/* If we are currently idle, try to start the next message. */
	if (si_sm_result == SI_SM_IDLE) {
		smi_inc_stat(smi_info, idles);

		si_sm_result = start_next_msg(smi_info);
		if (si_sm_result != SI_SM_IDLE)
			goto restart;
	}

	if ((si_sm_result == SI_SM_IDLE)
	    && (atomic_read(&smi_info->req_events))) {
		/*
		 * We are idle and the upper layer requested that I fetch
		 * events, so do so.
		 */
		atomic_set(&smi_info->req_events, 0);

		/*
		 * Take this opportunity to check the interrupt and
		 * message enable state for the BMC.  The BMC can be
		 * asynchronously reset, and may thus get interrupts
		 * disable and messages disabled.
		 */
		if (smi_info->supports_event_msg_buff || smi_info->irq) {
			start_check_enables(smi_info, true);
		} else {
			smi_info->curr_msg = alloc_msg_handle_irq(smi_info);
			if (!smi_info->curr_msg)
				goto out;

			start_getting_events(smi_info);
		}
		goto restart;
	}

	if (si_sm_result == SI_SM_IDLE && smi_info->timer_running) {
		/* Ok it if fails, the timer will just go off. */
		if (del_timer(&smi_info->si_timer))
			smi_info->timer_running = false;
	}

out:
	return si_sm_result;
}

static void check_start_timer_thread(struct smi_info *smi_info)
{
	if (smi_info->si_state == SI_NORMAL && smi_info->curr_msg == NULL) {
		smi_mod_timer(smi_info, jiffies + SI_TIMEOUT_JIFFIES);

		if (smi_info->thread)
			wake_up_process(smi_info->thread);

		start_next_msg(smi_info);
		smi_event_handler(smi_info, 0);
	}
}

static void flush_messages(void *send_info)
{
	struct smi_info *smi_info = send_info;
	enum si_sm_result result;

	/*
	 * Currently, this function is called only in run-to-completion
	 * mode.  This means we are single-threaded, no need for locks.
	 */
	result = smi_event_handler(smi_info, 0);
	while (result != SI_SM_IDLE) {
		udelay(SI_SHORT_TIMEOUT_USEC);
		result = smi_event_handler(smi_info, SI_SHORT_TIMEOUT_USEC);
	}
}

static void sender(void                *send_info,
		   struct ipmi_smi_msg *msg)
{
	struct smi_info   *smi_info = send_info;
	unsigned long     flags;

	debug_timestamp("Enqueue");

	if (smi_info->run_to_completion) {
		/*
		 * If we are running to completion, start it.  Upper
		 * layer will call flush_messages to clear it out.
		 */
		smi_info->waiting_msg = msg;
		return;
	}

	spin_lock_irqsave(&smi_info->si_lock, flags);
	/*
	 * The following two lines don't need to be under the lock for
	 * the lock's sake, but they do need SMP memory barriers to
	 * avoid getting things out of order.  We are already claiming
	 * the lock, anyway, so just do it under the lock to avoid the
	 * ordering problem.
	 */
	BUG_ON(smi_info->waiting_msg);
	smi_info->waiting_msg = msg;
	check_start_timer_thread(smi_info);
	spin_unlock_irqrestore(&smi_info->si_lock, flags);
}

static void set_run_to_completion(void *send_info, bool i_run_to_completion)
{
	struct smi_info   *smi_info = send_info;

	smi_info->run_to_completion = i_run_to_completion;
	if (i_run_to_completion)
		flush_messages(smi_info);
}

/*
 * Use -1 in the nsec value of the busy waiting timespec to tell that
 * we are spinning in kipmid looking for something and not delaying
 * between checks
 */
static inline void ipmi_si_set_not_busy(struct timespec64 *ts)
{
	ts->tv_nsec = -1;
}
static inline int ipmi_si_is_busy(struct timespec64 *ts)
{
	return ts->tv_nsec != -1;
}

static inline int ipmi_thread_busy_wait(enum si_sm_result smi_result,
					const struct smi_info *smi_info,
					struct timespec64 *busy_until)
{
	unsigned int max_busy_us = 0;

	if (smi_info->intf_num < num_max_busy_us)
		max_busy_us = kipmid_max_busy_us[smi_info->intf_num];
	if (max_busy_us == 0 || smi_result != SI_SM_CALL_WITH_DELAY)
		ipmi_si_set_not_busy(busy_until);
	else if (!ipmi_si_is_busy(busy_until)) {
		getnstimeofday64(busy_until);
		timespec64_add_ns(busy_until, max_busy_us*NSEC_PER_USEC);
	} else {
		struct timespec64 now;

		getnstimeofday64(&now);
		if (unlikely(timespec64_compare(&now, busy_until) > 0)) {
			ipmi_si_set_not_busy(busy_until);
			return 0;
		}
	}
	return 1;
}


/*
 * A busy-waiting loop for speeding up IPMI operation.
 *
 * Lousy hardware makes this hard.  This is only enabled for systems
 * that are not BT and do not have interrupts.  It starts spinning
 * when an operation is complete or until max_busy tells it to stop
 * (if that is enabled).  See the paragraph on kimid_max_busy_us in
 * Documentation/IPMI.txt for details.
 */
static int ipmi_thread(void *data)
{
	struct smi_info *smi_info = data;
	unsigned long flags;
	enum si_sm_result smi_result;
	struct timespec64 busy_until;

	ipmi_si_set_not_busy(&busy_until);
	set_user_nice(current, MAX_NICE);
	while (!kthread_should_stop()) {
		int busy_wait;

		spin_lock_irqsave(&(smi_info->si_lock), flags);
		smi_result = smi_event_handler(smi_info, 0);

		/*
		 * If the driver is doing something, there is a possible
		 * race with the timer.  If the timer handler see idle,
		 * and the thread here sees something else, the timer
		 * handler won't restart the timer even though it is
		 * required.  So start it here if necessary.
		 */
		if (smi_result != SI_SM_IDLE && !smi_info->timer_running)
			smi_mod_timer(smi_info, jiffies + SI_TIMEOUT_JIFFIES);

		spin_unlock_irqrestore(&(smi_info->si_lock), flags);
		busy_wait = ipmi_thread_busy_wait(smi_result, smi_info,
						  &busy_until);
		if (smi_result == SI_SM_CALL_WITHOUT_DELAY)
			; /* do nothing */
		else if (smi_result == SI_SM_CALL_WITH_DELAY && busy_wait)
			schedule();
		else if (smi_result == SI_SM_IDLE) {
			if (atomic_read(&smi_info->need_watch)) {
				schedule_timeout_interruptible(100);
			} else {
				/* Wait to be woken up when we are needed. */
				__set_current_state(TASK_INTERRUPTIBLE);
				schedule();
			}
		} else
			schedule_timeout_interruptible(1);
	}
	return 0;
}


static void poll(void *send_info)
{
	struct smi_info *smi_info = send_info;
	unsigned long flags = 0;
	bool run_to_completion = smi_info->run_to_completion;

	/*
	 * Make sure there is some delay in the poll loop so we can
	 * drive time forward and timeout things.
	 */
	udelay(10);
	if (!run_to_completion)
		spin_lock_irqsave(&smi_info->si_lock, flags);
	smi_event_handler(smi_info, 10);
	if (!run_to_completion)
		spin_unlock_irqrestore(&smi_info->si_lock, flags);
}

static void request_events(void *send_info)
{
	struct smi_info *smi_info = send_info;

	if (!smi_info->has_event_buffer)
		return;

	atomic_set(&smi_info->req_events, 1);
}

static void set_need_watch(void *send_info, bool enable)
{
	struct smi_info *smi_info = send_info;
	unsigned long flags;

	atomic_set(&smi_info->need_watch, enable);
	spin_lock_irqsave(&smi_info->si_lock, flags);
	check_start_timer_thread(smi_info);
	spin_unlock_irqrestore(&smi_info->si_lock, flags);
}

static void smi_timeout(unsigned long data)
{
	struct smi_info   *smi_info = (struct smi_info *) data;
	enum si_sm_result smi_result;
	unsigned long     flags;
	unsigned long     jiffies_now;
	long              time_diff;
	long		  timeout;

	spin_lock_irqsave(&(smi_info->si_lock), flags);
	debug_timestamp("Timer");

	jiffies_now = jiffies;
	time_diff = (((long)jiffies_now - (long)smi_info->last_timeout_jiffies)
		     * SI_USEC_PER_JIFFY);
	smi_result = smi_event_handler(smi_info, time_diff);

	if ((smi_info->irq) && (!smi_info->interrupt_disabled)) {
		/* Running with interrupts, only do long timeouts. */
		timeout = jiffies + SI_TIMEOUT_JIFFIES;
		smi_inc_stat(smi_info, long_timeouts);
		goto do_mod_timer;
	}

	/*
	 * If the state machine asks for a short delay, then shorten
	 * the timer timeout.
	 */
	if (smi_result == SI_SM_CALL_WITH_DELAY) {
		smi_inc_stat(smi_info, short_timeouts);
		timeout = jiffies + 1;
	} else {
		smi_inc_stat(smi_info, long_timeouts);
		timeout = jiffies + SI_TIMEOUT_JIFFIES;
	}

do_mod_timer:
	if (smi_result != SI_SM_IDLE)
		smi_mod_timer(smi_info, timeout);
	else
		smi_info->timer_running = false;
	spin_unlock_irqrestore(&(smi_info->si_lock), flags);
}

static int smi_start_processing(void       *send_info,
				ipmi_smi_t intf)
{
	struct smi_info *new_smi = send_info;
	int             enable = 0;

	new_smi->intf = intf;

	/* Set up the timer that drives the interface. */
	setup_timer(&new_smi->si_timer, smi_timeout, (long)new_smi);
	smi_mod_timer(new_smi, jiffies + SI_TIMEOUT_JIFFIES);

	/* Try to claim any interrupts. */
	if (new_smi->irq_setup)
		new_smi->irq_setup(new_smi);

	/*
	 * Check if the user forcefully enabled the daemon.
	 */
	if (new_smi->intf_num < num_force_kipmid)
		enable = force_kipmid[new_smi->intf_num];
	/*
	 * The BT interface is efficient enough to not need a thread,
	 * and there is no need for a thread if we have interrupts.
	 */
	else if ((new_smi->si_type != SI_BT) && (!new_smi->irq))
		enable = 1;

	if (enable) {
		new_smi->thread = kthread_run(ipmi_thread, new_smi,
					      "kipmi%d", new_smi->intf_num);
		if (IS_ERR(new_smi->thread)) {
			dev_notice(new_smi->dev, "Could not start"
				   " kernel thread due to error %ld, only using"
				   " timers to drive the interface\n",
				   PTR_ERR(new_smi->thread));
			new_smi->thread = NULL;
		}
	}

	return 0;
}

static int get_smi_info(void *send_info, struct ipmi_smi_info *data)
{
	struct smi_info *smi = send_info;

	data->addr_src = smi->addr_source;
	data->dev = smi->dev;
	data->addr_info = smi->addr_info;
	get_device(smi->dev);

	return 0;
}

static void set_maintenance_mode(void *send_info, bool enable)
{
	struct smi_info   *smi_info = send_info;

	if (!enable)
		atomic_set(&smi_info->req_events, 0);
}

static const struct ipmi_smi_handlers handlers = {
	.owner                  = THIS_MODULE,
	.start_processing       = smi_start_processing,
	.get_smi_info		= get_smi_info,
	.sender			= sender,
	.request_events		= request_events,
	.set_need_watch		= set_need_watch,
	.set_maintenance_mode   = set_maintenance_mode,
	.set_run_to_completion  = set_run_to_completion,
	.flush_messages		= flush_messages,
	.poll			= poll,
};

/*
 * There can be 4 IO ports passed in (with or without IRQs), 4 addresses,
 * a default IO port, and 1 ACPI/SPMI address.  That sets SI_MAX_DRIVERS.
 */
static int smi_num; /* Used to sequence the SMIs */

#define DEFAULT_REGSPACING	1
#define DEFAULT_REGSIZE		1

#define IPMI_IO_ADDR_SPACE  0
#define IPMI_MEM_ADDR_SPACE 1
static const char * const addr_space_to_str[] = { "i/o", "mem" };

module_param_array(force_kipmid, int, &num_force_kipmid, 0);
MODULE_PARM_DESC(force_kipmid, "Force the kipmi daemon to be enabled (1) or"
		 " disabled(0).  Normally the IPMI driver auto-detects"
		 " this, but the value may be overridden by this parm.");
module_param_array(kipmid_max_busy_us, uint, &num_max_busy_us, 0644);
MODULE_PARM_DESC(kipmid_max_busy_us,
		 "Max time (in microseconds) to busy-wait for IPMI data before"
		 " sleeping. 0 (default) means to wait forever. Set to 100-500"
		 " if kipmid is using up a lot of CPU time.");

static int wait_for_msg_done(struct smi_info *smi_info)
{
	enum si_sm_result     smi_result;

	smi_result = smi_info->handlers->event(smi_info->si_sm, 0);
	for (;;) {
		if (smi_result == SI_SM_CALL_WITH_DELAY ||
		    smi_result == SI_SM_CALL_WITH_TICK_DELAY) {
			schedule_timeout_uninterruptible(1);
			smi_result = smi_info->handlers->event(
				smi_info->si_sm, jiffies_to_usecs(1));
		} else if (smi_result == SI_SM_CALL_WITHOUT_DELAY) {
			smi_result = smi_info->handlers->event(
				smi_info->si_sm, 0);
		} else
			break;
	}
	if (smi_result == SI_SM_HOSED)
		/*
		 * We couldn't get the state machine to run, so whatever's at
		 * the port is probably not an IPMI SMI interface.
		 */
		return -ENODEV;

	return 0;
}

static int try_get_dev_id(struct smi_info *smi_info)
{
	unsigned char         msg[2];
	unsigned char         *resp;
	unsigned long         resp_len;
	int                   rv = 0;

	resp = kmalloc(IPMI_MAX_MSG_LENGTH, GFP_KERNEL);
	if (!resp)
		return -ENOMEM;

	/*
	 * Do a Get Device ID command, since it comes back with some
	 * useful info.
	 */
	msg[0] = IPMI_NETFN_APP_REQUEST << 2;
	msg[1] = IPMI_GET_DEVICE_ID_CMD;
	smi_info->handlers->start_transaction(smi_info->si_sm, msg, 2);

	rv = wait_for_msg_done(smi_info);
	if (rv)
		goto out;

	resp_len = smi_info->handlers->get_result(smi_info->si_sm,
						  resp, IPMI_MAX_MSG_LENGTH);

	/* Check and record info from the get device id, in case we need it. */
	rv = ipmi_demangle_device_id(resp, resp_len, &smi_info->device_id);

out:
	kfree(resp);
	return rv;
}

static int try_enable_event_buffer(struct smi_info *smi_info)
{
	unsigned char         msg[3];
	unsigned char         *resp;
	unsigned long         resp_len;
	int                   rv = 0;

	resp = kmalloc(IPMI_MAX_MSG_LENGTH, GFP_KERNEL);
	if (!resp)
		return -ENOMEM;

	msg[0] = IPMI_NETFN_APP_REQUEST << 2;
	msg[1] = IPMI_GET_BMC_GLOBAL_ENABLES_CMD;
	smi_info->handlers->start_transaction(smi_info->si_sm, msg, 2);

	rv = wait_for_msg_done(smi_info);
	if (rv) {
		printk(KERN_WARNING PFX "Error getting response from get"
		       " global enables command, the event buffer is not"
		       " enabled.\n");
		goto out;
	}

	resp_len = smi_info->handlers->get_result(smi_info->si_sm,
						  resp, IPMI_MAX_MSG_LENGTH);

	if (resp_len < 4 ||
			resp[0] != (IPMI_NETFN_APP_REQUEST | 1) << 2 ||
			resp[1] != IPMI_GET_BMC_GLOBAL_ENABLES_CMD   ||
			resp[2] != 0) {
		printk(KERN_WARNING PFX "Invalid return from get global"
		       " enables command, cannot enable the event buffer.\n");
		rv = -EINVAL;
		goto out;
	}

	if (resp[3] & IPMI_BMC_EVT_MSG_BUFF) {
		/* buffer is already enabled, nothing to do. */
		smi_info->supports_event_msg_buff = true;
		goto out;
	}

	msg[0] = IPMI_NETFN_APP_REQUEST << 2;
	msg[1] = IPMI_SET_BMC_GLOBAL_ENABLES_CMD;
	msg[2] = resp[3] | IPMI_BMC_EVT_MSG_BUFF;
	smi_info->handlers->start_transaction(smi_info->si_sm, msg, 3);

	rv = wait_for_msg_done(smi_info);
	if (rv) {
		printk(KERN_WARNING PFX "Error getting response from set"
		       " global, enables command, the event buffer is not"
		       " enabled.\n");
		goto out;
	}

	resp_len = smi_info->handlers->get_result(smi_info->si_sm,
						  resp, IPMI_MAX_MSG_LENGTH);

	if (resp_len < 3 ||
			resp[0] != (IPMI_NETFN_APP_REQUEST | 1) << 2 ||
			resp[1] != IPMI_SET_BMC_GLOBAL_ENABLES_CMD) {
		printk(KERN_WARNING PFX "Invalid return from get global,"
		       "enables command, not enable the event buffer.\n");
		rv = -EINVAL;
		goto out;
	}

	if (resp[2] != 0)
		/*
		 * An error when setting the event buffer bit means
		 * that the event buffer is not supported.
		 */
		rv = -ENOENT;
	else
		smi_info->supports_event_msg_buff = true;

out:
	kfree(resp);
	return rv;
}

static int smi_type_proc_show(struct seq_file *m, void *v)
{
	struct smi_info *smi = m->private;

	seq_printf(m, "%s\n", si_to_str[smi->si_type]);

	return 0;
}

static int smi_type_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, smi_type_proc_show, PDE_DATA(inode));
}

static const struct file_operations smi_type_proc_ops = {
	.open		= smi_type_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int smi_si_stats_proc_show(struct seq_file *m, void *v)
{
	struct smi_info *smi = m->private;

	seq_printf(m, "interrupts_enabled:    %d\n",
		       smi->irq && !smi->interrupt_disabled);
	seq_printf(m, "short_timeouts:        %u\n",
		       smi_get_stat(smi, short_timeouts));
	seq_printf(m, "long_timeouts:         %u\n",
		       smi_get_stat(smi, long_timeouts));
	seq_printf(m, "idles:                 %u\n",
		       smi_get_stat(smi, idles));
	seq_printf(m, "interrupts:            %u\n",
		       smi_get_stat(smi, interrupts));
	seq_printf(m, "attentions:            %u\n",
		       smi_get_stat(smi, attentions));
	seq_printf(m, "flag_fetches:          %u\n",
		       smi_get_stat(smi, flag_fetches));
	seq_printf(m, "hosed_count:           %u\n",
		       smi_get_stat(smi, hosed_count));
	seq_printf(m, "complete_transactions: %u\n",
		       smi_get_stat(smi, complete_transactions));
	seq_printf(m, "events:                %u\n",
		       smi_get_stat(smi, events));
	seq_printf(m, "watchdog_pretimeouts:  %u\n",
		       smi_get_stat(smi, watchdog_pretimeouts));
	seq_printf(m, "incoming_messages:     %u\n",
		       smi_get_stat(smi, incoming_messages));
	return 0;
}

static int smi_si_stats_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, smi_si_stats_proc_show, PDE_DATA(inode));
}

static const struct file_operations smi_si_stats_proc_ops = {
	.open		= smi_si_stats_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int smi_params_proc_show(struct seq_file *m, void *v)
{
	struct smi_info *smi = m->private;

	seq_printf(m,
		   "%s,%s,0x%lx,rsp=%d,rsi=%d,rsh=%d,irq=%d,ipmb=%d\n",
		   si_to_str[smi->si_type],
		   addr_space_to_str[smi->io.addr_type],
		   smi->io.addr_data,
		   smi->io.regspacing,
		   smi->io.regsize,
		   smi->io.regshift,
		   smi->irq,
		   smi->slave_addr);

	return 0;
}

static int smi_params_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, smi_params_proc_show, PDE_DATA(inode));
}

static const struct file_operations smi_params_proc_ops = {
	.open		= smi_params_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static inline void wait_for_timer_and_thread(struct smi_info *smi_info)
{
	if (smi_info->thread != NULL)
		kthread_stop(smi_info->thread);
	if (smi_info->timer_running)
		del_timer_sync(&smi_info->si_timer);
}

static int try_smi_init(struct smi_info *new_smi)
{
	int rv = 0;
	int i;

	printk(KERN_INFO PFX "Trying %s-specified %s state"
	       " machine at %s address 0x%lx, slave address 0x%x,"
	       " irq %d\n",
	       ipmi_addr_src_to_str(new_smi->addr_source),
	       si_to_str[new_smi->si_type],
	       addr_space_to_str[new_smi->io.addr_type],
	       new_smi->io.addr_data,
	       new_smi->slave_addr, new_smi->irq);

	switch (new_smi->si_type) {
#if 0
	case SI_KCS:
		new_smi->handlers = &kcs_smi_handlers;
		break;

	case SI_SMIC:
		new_smi->handlers = &smic_smi_handlers;
		break;
#endif
	case SI_BT:
#ifdef USE_LPC_IPMI_SI_SM
		new_smi->handlers = &lpc_bt_handlers;
#else
		new_smi->handlers = &bt_smi_handlers;
#endif
		break;

	default:
		/* No support for anything else yet. */
		rv = -EIO;
		goto out_err;
	}

	/* Allocate the state machine's data and initialize it. */
	new_smi->si_sm = kmalloc(new_smi->handlers->size(), GFP_KERNEL);
	if (!new_smi->si_sm) {
		printk(KERN_ERR PFX
		       "Could not allocate state machine memory\n");
		rv = -ENOMEM;
		goto out_err;
	}
	new_smi->io_size = new_smi->handlers->init_data(new_smi->si_sm,
							&new_smi->io);

	/* Now that we know the I/O size, we can set up the I/O. */
	rv = new_smi->io_setup(new_smi);
	if (rv) {
		printk(KERN_ERR PFX "Could not set up I/O space\n");
		goto out_err;
	}

	/* Do low-level detection first. */
	if (new_smi->handlers->detect(new_smi->si_sm)) {
		if (new_smi->addr_source)
			printk(KERN_INFO PFX "Interface detection failed\n");
		rv = -ENODEV;
		goto out_err;
	}

	/*
	 * Attempt a get device id command.  If it fails, we probably
	 * don't have a BMC here.
	 */
	rv = try_get_dev_id(new_smi);
	if (rv) {
		if (new_smi->addr_source)
			printk(KERN_INFO PFX "There appears to be no BMC"
			       " at this location\n");
		goto out_err;
	}

	new_smi->waiting_msg = NULL;
	new_smi->curr_msg = NULL;
	atomic_set(&new_smi->req_events, 0);
	new_smi->run_to_completion = false;
	for (i = 0; i < SI_NUM_STATS; i++)
		atomic_set(&new_smi->stats[i], 0);

	new_smi->interrupt_disabled = true;
	atomic_set(&new_smi->need_watch, 0);
	new_smi->intf_num = smi_num;
	smi_num++;

	rv = try_enable_event_buffer(new_smi);
	if (rv == 0)
		new_smi->has_event_buffer = true;

	/*
	 * Start clearing the flags before we enable interrupts or the
	 * timer to avoid racing with the timer.
	 */
	start_clear_flags(new_smi, false);

	/*
	 * IRQ is defined to be set when non-zero.  req_events will
	 * cause a global flags check that will enable interrupts.
	 */
	if (new_smi->irq) {
		new_smi->interrupt_disabled = false;
		atomic_set(&new_smi->req_events, 1);
	}

	rv = ipmi_register_smi(&handlers,
			       new_smi,
			       &new_smi->device_id,
			       new_smi->dev,
			       new_smi->slave_addr);
	if (rv) {
		dev_err(new_smi->dev, "Unable to register device: error %d\n",
			rv);
		goto out_err_stop_timer;
	}

	rv = ipmi_smi_add_proc_entry(new_smi->intf, "type",
				     &smi_type_proc_ops,
				     new_smi);
	if (rv) {
		dev_err(new_smi->dev, "Unable to create proc entry: %d\n", rv);
		goto out_err_stop_timer;
	}

	rv = ipmi_smi_add_proc_entry(new_smi->intf, "si_stats",
				     &smi_si_stats_proc_ops,
				     new_smi);
	if (rv) {
		dev_err(new_smi->dev, "Unable to create proc entry: %d\n", rv);
		goto out_err_stop_timer;
	}

	rv = ipmi_smi_add_proc_entry(new_smi->intf, "params",
				     &smi_params_proc_ops,
				     new_smi);
	if (rv) {
		dev_err(new_smi->dev, "Unable to create proc entry: %d\n", rv);
		goto out_err_stop_timer;
	}

	dev_info(new_smi->dev, "IPMI %s interface initialized\n",
		 si_to_str[new_smi->si_type]);

	return 0;

out_err_stop_timer:
	wait_for_timer_and_thread(new_smi);

out_err:
	new_smi->interrupt_disabled = true;

	if (new_smi->intf) {
		ipmi_smi_t intf = new_smi->intf;
		new_smi->intf = NULL;
		ipmi_unregister_smi(intf);
	}

	if (new_smi->irq_cleanup) {
		new_smi->irq_cleanup(new_smi);
		new_smi->irq_cleanup = NULL;
	}

	/*
	 * Wait until we know that we are out of any interrupt
	 * handlers might have been running before we freed the
	 * interrupt.
	 */
	synchronize_sched();

	if (new_smi->si_sm) {
		if (new_smi->handlers)
			new_smi->handlers->cleanup(new_smi->si_sm);
		kfree(new_smi->si_sm);
		new_smi->si_sm = NULL;
	}
	if (new_smi->addr_source_cleanup) {
		new_smi->addr_source_cleanup(new_smi);
		new_smi->addr_source_cleanup = NULL;
	}
	if (new_smi->io_cleanup) {
		new_smi->io_cleanup(new_smi);
		new_smi->io_cleanup = NULL;
	}

	if (new_smi->dev_registered) {
		platform_device_unregister(new_smi->pdev);
		new_smi->dev_registered = false;
	}

	return rv;
}

/*
 * There following codes are from ipmi_si driver, but with some necessary
 * chagnes for Hip06 LPC.
 */

static unsigned char intf_lpc_inb(const struct si_sm_io *io,
				  unsigned int offset)
{
	unsigned long ptaddr;
	struct smi_info *info = to_smi_info(io);
	struct lpc_dev_data *lpc_data = dev_get_platdata(info->dev);
	struct extio_ops *ops = lpc_data->hostops;

	if (!ops->pfin || offset >= lpc_data->iosize) {
		dev_err_once(info->dev, "no pfin or offset(%d) is too big!\n",
			offset);
		return -1;
	}

	if (info->intf)
		printk_once("%s intf_num=%d startIO=0x%lx\n", __func__,
			info->intf_num, io->addr_data);

	ptaddr = io->addr_data + offset * io->regspacing;

	return ops->pfin(ops->devpara, ptaddr, 1);
}

static void intf_lpc_outb(const struct si_sm_io *io, unsigned int offset,
			  unsigned char val)
{
	unsigned long ptaddr;
	struct smi_info *info = to_smi_info(io);
	struct lpc_dev_data *lpc_data = dev_get_platdata(info->dev);
	struct extio_ops *ops = lpc_data->hostops;

	if (!ops->pfout || offset >= lpc_data->iosize) {
		dev_err_once(info->dev, "no pfout or offset(%d) is too big!\n",
			offset);
		return;
	}

	if (info->intf)
		printk_once("%s infoNo=%d startIO=0x%lx\n", __func__,
			info->intf_num,	io->addr_data);

	ptaddr = io->addr_data + offset * io->regspacing;
	ops->pfout(ops->devpara, ptaddr, val, sizeof(val));
}

static int lpc_io_setup(struct smi_info *info)
{
	struct lpc_dev_data *lpc_data = dev_get_platdata(info->dev);

	if (!lpc_data || !lpc_data->hostops) {
		dev_err(info->dev, "no binding I/O operation!\n");
		return -ENODEV;
	}

	info->io.inputb = intf_lpc_inb;
	info->io.outputb = intf_lpc_outb;

	return 0;
}

static const struct of_device_id of_ipmi_match[] = {
	{ .name = "hisi-bt", .type = "ipmi", .compatible = "ipmi-bt",
	  .data = (void *)(unsigned long) SI_BT },
	{},
};

static const struct acpi_device_id acpi_ipmi_match[] = {
	{ "IPI0001", 0 },
	{ },
};

#ifdef CONFIG_OF
static int of_ipmi_probe(struct platform_device *dev)
{
	const struct of_device_id *match;
	struct lpc_dev_data *lpc_data;
	struct smi_info *info;
	struct device_node *np = dev->dev.of_node;
	int ret;

	dev_info(&dev->dev, "probing via device tree\n");

	match = of_match_device(of_ipmi_match, &dev->dev);
	if (!match)
		return -ENODEV;

	if (!of_device_is_available(np))
		return -EINVAL;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&dev->dev,
			"could not allocate memory for OF probe\n");
		return -ENOMEM;
	}
	spin_lock_init(&info->si_lock);

	info->si_type		= (enum si_type) match->data;
	info->addr_source	= SI_DEVICETREE;

	/* only support byte I/O for this device. */
	info->io.regsize	= DEFAULT_REGSIZE;
	info->io.regspacing	= DEFAULT_REGSPACING;
	info->io.regshift	= 0;

	/* get the device address. */
	lpc_data = dev_get_platdata(&dev->dev);
	if (!lpc_data || !lpc_data->start_adr || !lpc_data->iosize) {
		dev_err(&dev->dev, "No any platform data!\n");
		return -ENXIO;
	}
	info->io_setup		= lpc_io_setup;
	info->io.addr_type	= IPMI_IO_ADDR_SPACE;
	info->io.addr_data	= lpc_data->start_adr;

	info->irq		= 0;
	info->irq_setup		= NULL;

	info->dev		= &dev->dev;
	info->pdev		= dev;
	info->ops		= lpc_data->hostops;

	dev_dbg(&dev->dev, "addr 0x%lx regsize %d spacing %d irq %d\n",
		info->io.addr_data, info->io.regsize, info->io.regspacing,
		info->irq);

	dev_set_drvdata(&dev->dev, info);

	ret = try_smi_init(info);
	if (ret) {
		kfree(info);
		return ret;
	}
	return 0;
}
#else
static int of_ipmi_probe(struct platform_device *dev)
{
	return -ENODEV;
}
#endif

#ifdef CONFIG_ACPI
static int acpi_ipmi_probe(struct platform_device *dev)
{
	struct lpc_dev_data lpc_data;
	struct resource *res;
	unsigned long long tmp;
	struct smi_info *info;
	acpi_handle handle;
	acpi_status status;
	int rv = -EINVAL;

	handle = ACPI_HANDLE(&dev->dev);
	if (!handle)
		return -ENODEV;

	/*
	 * It is possible thesre are  multiple ipmi interface devices.
	 * After this lpc-ipmi ko is loaded, there are two drivers for ipmi si
	 * devices now. It is risky non-hisi-lpc ipmi device try to bind with
	 * this driver probing. Here add a checking to filter these possible
	 * devices.
	 */
	if (!acpi_driver_match_device(dev->dev.parent,
			&hisilpc_driver.driver)) {
		dev_err(&dev->dev, "Not hisi IPMI!\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	spin_lock_init(&info->si_lock);

	info->addr_source = SI_ACPI;
	dev_info(&dev->dev, PFX "probing via ACPI\n");

	info->addr_info.acpi_info.acpi_handle = handle;

	/* _IFT tells us the interface type: KCS, BT, etc */
	status = acpi_evaluate_integer(handle, "_IFT", NULL, &tmp);
	if (ACPI_FAILURE(status)) {
		dev_err(&dev->dev, "Could not find ACPI IPMI interface type\n");
		goto err_free;
	}
	switch (tmp) {
	case 1:
		info->si_type = SI_KCS;
		break;
	case 2:
		info->si_type = SI_SMIC;
		break;
	case 3:
		info->si_type = SI_BT;
		break;
	case 4: /* SSIF, just ignore */
		rv = -ENODEV;
		goto err_free;
	default:
		dev_info(&dev->dev, "unknown IPMI type %lld\n", tmp);
		goto err_free;
	}

	info->io.regsize = DEFAULT_REGSPACING;
	info->io.regspacing = DEFAULT_REGSPACING;
	info->io.regshift = 0;

	res = platform_get_resource(dev, IORESOURCE_IO, 0);
	if (!res) {
		dev_err(&dev->dev, "no I/O or memory address\n");
		lpc_data.start_adr = LPC_IPMI_ADDR;
		lpc_data.iosize = LPC_IPMI_IOREG_SIZE;
	} else {
		lpc_data.start_adr = res->start;
		lpc_data.iosize = resource_size(res);
	}
	lpc_data.hostops = &hisi_lpc_ops;
	platform_device_add_data(dev, &lpc_data, sizeof(lpc_data));

	info->io_setup		= lpc_io_setup;
	info->io.addr_type	= IPMI_IO_ADDR_SPACE;
	info->io.addr_data	= lpc_data.start_adr;

	info->irq = 0;
	info->irq_setup = NULL;

	info->dev = &dev->dev;
	info->ops = lpc_data.hostops;

	platform_set_drvdata(dev, info);

	dev_info(info->dev, "ACPI: addr 0x%lx regsize %d spacing %d irq %d\n",
		 info->io.addr_data, info->io.regsize, info->io.regspacing,
		 info->irq);
	/* Don't intoduce a list here.. */
	rv = try_smi_init(info);
	if (rv)
		kfree(info);

	return rv;

err_free:
	kfree(info);
	return rv;
}
#else
static int acpi_ipmi_probe(struct platform_device *dev)
{
	return -ENODEV;
}
#endif

static int ipmi_probe(struct platform_device *dev)
{
	if (of_ipmi_probe(dev) == 0)
		return 0;

	return acpi_ipmi_probe(dev);
}

static int ipmi_remove(struct platform_device *dev)
{
	struct smi_info *info = dev_get_drvdata(&dev->dev);

	cleanup_one_si(info);
	return 0;
}

static struct platform_driver ipmi_driver = {
	.driver = {
		.name = "hisi_ipmi_si",
		.of_match_table = of_ipmi_match,
		.acpi_match_table = ACPI_PTR(acpi_ipmi_match),
	},
	.probe		= ipmi_probe,
	.remove		= ipmi_remove,
};

static void cleanup_one_si(struct smi_info *to_clean)
{
	int           rv = 0;

	if (!to_clean)
		return;

	if (to_clean->intf) {
		ipmi_smi_t intf = to_clean->intf;

		to_clean->intf = NULL;
		rv = ipmi_unregister_smi(intf);
		if (rv) {
			pr_err(PFX "Unable to unregister device: errno=%d\n",
			       rv);
		}
	}

	if (to_clean->dev)
		dev_set_drvdata(to_clean->dev, NULL);

	/* list_del(&to_clean->link); */

	/*
	 * Make sure that interrupts, the timer and the thread are
	 * stopped and will not run again.
	 */
	if (to_clean->irq_cleanup)
		to_clean->irq_cleanup(to_clean);
	wait_for_timer_and_thread(to_clean);

	/*
	 * Timeouts are stopped, now make sure the interrupts are off
	 * in the BMC.  Note that timers and CPU interrupts are off,
	 * so no need for locks.
	 */
	while (to_clean->curr_msg || (to_clean->si_state != SI_NORMAL)) {
		poll(to_clean);
		schedule_timeout_uninterruptible(1);
	}
	disable_si_irq(to_clean, false);
	while (to_clean->curr_msg || (to_clean->si_state != SI_NORMAL)) {
		poll(to_clean);
		schedule_timeout_uninterruptible(1);
	}

	if (to_clean->handlers)
		to_clean->handlers->cleanup(to_clean->si_sm);

	kfree(to_clean->si_sm);

	if (to_clean->addr_source_cleanup)
		to_clean->addr_source_cleanup(to_clean);
	if (to_clean->io_cleanup)
		to_clean->io_cleanup(to_clean);

	if (to_clean->dev_registered)
		platform_device_unregister(to_clean->pdev);

	kfree(to_clean);
}
/* End of ipmi processing. */

/*
 * hisilpc_probe - the probe callback function for hisi lpc device,
 *		   will finish all the initialization.
 * @pdev: the platform device corresponding to hisi lpc
 *
 * Returns 0 on success, non-zero on fail.
 *
 */
static int hisilpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hisilpc_dev *lpcdev;
	struct resource *res;
	int ret = 0;

	dev_info(dev, "probing...\n");

	lpcdev = devm_kzalloc(dev, sizeof(struct hisilpc_dev), GFP_KERNEL);
	if (!lpcdev)
		return -ENOMEM;

	spin_lock_init(&lpcdev->cycle_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no MEM resource\n");
		return -ENODEV;
	}

	lpcdev->membase = devm_ioremap_resource(dev, res);
	if (IS_ERR(lpcdev->membase)) {
		dev_err(dev, "remap failed\n");
		return PTR_ERR(lpcdev->membase);
	}

	/* intialize the extio_node of this hisi-lpc bus device */
	lpcdev->ops = &hisi_lpc_ops;
	lpcdev->ops->devpara = lpcdev;

	platform_set_drvdata(pdev, lpcdev);

	device_unlock(dev);
	/*
	 * register IPMI driver before the corresponding driver creation,
	 * this way can avoid the error handling.
	 */
	ret = platform_driver_register(&ipmi_driver);
	if (ret) {
		dev_err(dev, "Can not register hisi IPMI driver: %d\n", -ret);
		return ret;
	}
	device_lock(dev);

	if (!ret)
		dev_info(dev, "hslpc end probing..\n");
	else
		dev_info(dev, "hslpc probing is fail(%d)\n", ret);

	return ret;
}

static int hisilpc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_info(&pdev->dev, "hisilpc driver removing...\n");
	device_unlock(dev);
	platform_driver_unregister(&ipmi_driver);
	device_lock(dev);

	return 0;
}

static const struct of_device_id hisilpc_of_match[] = {
	{
		.compatible = "hisilicon,hip06-lpc",
		.compatible = "hisilicon,hip07-lpc",
	},
	{},
};

static const struct acpi_device_id hisilpc_acpi_match[] = {
	{"HISI0191", },
	{},
};

static struct platform_driver hisilpc_driver = {
	.driver = {
		.name = "hisi_lpc",
		.of_match_table = hisilpc_of_match,
		.acpi_match_table = hisilpc_acpi_match,
	},
	.probe = hisilpc_probe,
	.remove = hisilpc_remove,
};

module_platform_driver(hisilpc_driver);

MODULE_DESCRIPTION("Hip06/07 LPC IPMI driver");
MODULE_AUTHOR("Zhichang Yuan");
MODULE_LICENSE("GPL");
