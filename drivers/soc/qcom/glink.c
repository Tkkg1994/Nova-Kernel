/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/err.h>
#include <linux/ipc_logging.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/rwsem.h>
#include <soc/qcom/glink.h>
#include "glink_core_if.h"
#include "glink_private.h"
#include "glink_xprt_if.h"

/* Number of internal IPC Logging log pages */
#define NUM_LOG_PAGES	3

/**
 * struct glink_core_xprt_ctx - transport representation structure
 * @list_node:			used to chain this transport in a global
 *				transport list
 * @name:			name of this transport
 * @edge:			what this transport connects to
 * @versions:			array of transport versions this implementation
 *				supports
 * @versions_entries:		number of entries in @versions
 * @local_version_idx:		local version index into @versions this
 *				transport is currently running
 * @remote_version_idx:		remote version index into @versions this
 *				transport is currently running
 * @capabilities:		Capabilities of underlying transport
 * @ops:			transport defined implementation of common
 *				operations
 * @local_state:		value from local_channel_state_e representing
 *				the local state of this transport
 * @remote_neg_completed:	is the version negotiation with the remote end
 *				completed
 * @xprt_ctx_lock_lhb1		lock to protect @next_lcid and @channels
 * @next_lcid:			logical channel identifier to assign to the next
 *				created channel
 * @max_cid:			maximum number of channel identifiers supported
 * @max_iid:			maximum number of intent identifiers supported
 * @tx_work:			work item to process @tx_ready
 * @tx_wq:			workqueue to run @tx_work
 * @channels:			list of all existing channels on this transport
 * @tx_ready_mutex_lhb2:	lock to protect @tx_ready
 * @tx_ready:			list of all channels ready to transmit
 */
struct glink_core_xprt_ctx {
	struct list_head list_node;
	char name[GLINK_NAME_SIZE];
	char edge[GLINK_NAME_SIZE];
	const struct glink_core_version *versions;
	size_t versions_entries;
	uint32_t local_version_idx;
	uint32_t remote_version_idx;
	uint32_t capabilities;
	struct glink_transport_if *ops;
	enum transport_state_e local_state;
	bool remote_neg_completed;

	spinlock_t xprt_ctx_lock_lhb1;
	struct list_head channels;
	uint32_t next_lcid;

	uint32_t max_cid;
	uint32_t max_iid;
	struct work_struct tx_work;
	struct workqueue_struct *tx_wq;

	struct mutex tx_ready_mutex_lhb2;
	struct list_head tx_ready;
};

/**
 * Channel Context
 * @port_list_node:	channel list node used by transport "channels" list
 * @tx_ready_list_node:	channels that have data ready to transmit
 * @name:		name of the channel
 *
 * @user_priv:		user opaque data type passed into glink_open()
 * @notify_rx:		RX notification function
 * @notify_tx_done:	TX-done notification function (remote side is done)
 * @notify_state:	Channel state (connected / disconnected) notifications
 * @notify_rx_intent_req: Request from remote side for an intent
 * @notify_rxv:		RX notification function (for io buffer chain)
 * @notify_rx_sigs:	RX signal change notification
 *
 * @transport_ptr:	Transport this channel uses
 * @lcid:		Local channel ID
 * @rcid:		Remote channel ID
 * @local_open_state:	Local channel state
 * @remote_opened:	Remote channel state (opened or closed)
 * @int_req_ack:	Remote side intent request ACK state
 * @int_req_ack_complete: Intent tracking completion - received remote ACK
 * @int_req_complete:	Intent tracking completion - received intent
 *
 * @local_rx_intent_lst_lock_lhc1:	RX intent list lock
 * @local_rx_intent_list:		Active RX Intents queued by client
 * @local_rx_intent_ntfy_list:		Client notified, waiting for rx_done()
 * @local_rx_intent_free_list:		Available intent container structure
 *
 * @rmt_rx_intent_lst_lock_lhc2:	Remote RX intent list lock
 * @rmt_rx_intent_list:			Remote RX intent list
 *
 * @max_used_liid:			Maximum Local Intent ID used
 * @dummy_riid:				Dummy remote intent ID
 *
 * @tx_lists_mutex_lhc3:		TX list lock
 * @tx_active:				Ready to transmit
 * @tx_pending_remote_done:		Transmitted, waiting for remote done
 * @lsigs:				Local signals
 * @rsigs:				Remote signals
 */
struct channel_ctx {
	struct list_head port_list_node;
	struct list_head tx_ready_list_node;
	char name[GLINK_NAME_SIZE];

	/* user info */
	void *user_priv;
	void (*notify_rx)(void *handle, const void *priv, const void *pkt_priv,
			const void *ptr, size_t size);
	void (*notify_tx_done)(void *handle, const void *priv,
			const void *pkt_priv, const void *ptr);
	void (*notify_state)(void *handle, const void *priv, unsigned event);
	bool (*notify_rx_intent_req)(void *handle, const void *priv,
			size_t req_size);
	void (*notify_rxv)(void *handle, const void *priv, const void *pkt_priv,
			   void *iovec, size_t size,
			   void * (*vbuf_provider)(void *iovec, size_t offset,
						  size_t *size),
			   void * (*pbuf_provider)(void *iovec, size_t offset,
						  size_t *size));
	void (*notify_rx_sigs)(void *handle, const void *priv,
			uint32_t old_sigs, uint32_t new_sigs);
	void (*notify_rx_abort)(void *handle, const void *priv,
				const void *pkt_priv);
	void (*notify_tx_abort)(void *handle, const void *priv,
				const void *pkt_priv);

	/* internal port state */
	struct glink_core_xprt_ctx *transport_ptr;
	uint32_t lcid;
	uint32_t rcid;
	enum local_channel_state_e local_open_state;
	bool remote_opened;
	bool int_req_ack;
	struct completion int_req_ack_complete;
	struct completion int_req_complete;

	spinlock_t local_rx_intent_lst_lock_lhc1;
	struct list_head local_rx_intent_list;
	struct list_head local_rx_intent_ntfy_list;
	struct list_head local_rx_intent_free_list;

	spinlock_t rmt_rx_intent_lst_lock_lhc2;
	struct list_head rmt_rx_intent_list;

	uint32_t max_used_liid;
	uint32_t dummy_riid;

	struct mutex tx_lists_mutex_lhc3;
	struct list_head tx_active;
	struct list_head tx_pending_remote_done;

	uint32_t lsigs;
	uint32_t rsigs;
};

static struct glink_core_if core_impl;
static void *log_ctx;
static unsigned glink_debug_mask = QCOM_GLINK_INFO;
module_param_named(debug_mask, glink_debug_mask,
		   uint, S_IRUGO | S_IWUSR | S_IWGRP);

static LIST_HEAD(transport_list);

/*
 * Used while notifying the clients about link state events. Since the clients
 * need to store the callback information temporarily and since all the
 * existing accesses to transport list are in non-IRQ context, defining the
 * transport_list_lock as a mutex.
 */
static DEFINE_MUTEX(transport_list_lock_lha0);

struct link_state_notifier_info {
	struct list_head list;
	char transport[GLINK_NAME_SIZE];
	char edge[GLINK_NAME_SIZE];
	void (*glink_link_state_notif_cb)(
			struct glink_link_state_cb_info *cb_info, void *priv);
	void *priv;
};
static LIST_HEAD(link_state_notifier_list);
static DEFINE_MUTEX(link_state_notifier_lock_lha1);

static struct glink_core_xprt_ctx *find_open_transport(const char *edge,
						       const char *name);

static bool xprt_is_fully_opened(struct glink_core_xprt_ctx *xprt);

static struct channel_ctx *xprt_lcid_to_ch_ctx(
					struct glink_core_xprt_ctx *xprt_ctx,
					uint32_t lcid);

static struct channel_ctx *xprt_rcid_to_ch_ctx(
					struct glink_core_xprt_ctx *xprt_ctx,
					uint32_t rcid);

static void xprt_schedule_tx(struct glink_core_xprt_ctx *xprt_ptr,
			     struct channel_ctx *ch_ptr,
			     struct glink_core_tx_pkt *tx_info);

static int xprt_single_threaded_tx(struct glink_core_xprt_ctx *xprt_ptr,
			     struct channel_ctx *ch_ptr,
			     struct glink_core_tx_pkt *tx_info);

static void tx_work_func(struct work_struct *work);

static struct channel_ctx *ch_name_to_ch_ctx_create(
					struct glink_core_xprt_ctx *xprt_ctx,
					const char *name);

static void ch_push_remote_rx_intent(struct channel_ctx *ctx, size_t size,
							uint32_t riid);

static int ch_pop_remote_rx_intent(struct channel_ctx *ctx, size_t size,
							uint32_t *riid_ptr);

static struct glink_core_rx_intent *ch_push_local_rx_intent(
		struct channel_ctx *ctx, const void *pkt_priv, size_t size);

static void ch_remove_local_rx_intent(struct channel_ctx *ctx, uint32_t liid);

static struct glink_core_rx_intent *ch_get_local_rx_intent(
		struct channel_ctx *ctx, uint32_t liid);

static void ch_set_local_rx_intent_notified(struct channel_ctx *ctx,
				struct glink_core_rx_intent *intent_ptr);

static struct glink_core_rx_intent *ch_get_local_rx_intent_notified(
		struct channel_ctx *ctx, const void *ptr);

static void ch_remove_local_rx_intent_notified(struct channel_ctx *ctx,
					struct glink_core_rx_intent *liid_ptr);

static struct glink_core_rx_intent *ch_get_free_local_rx_intent(
		struct channel_ctx *ctx);

static void ch_purge_intent_lists(struct channel_ctx *ctx);

static void ch_add_rcid(struct glink_core_xprt_ctx *xprt_ctx,
			struct channel_ctx *ctx,
			uint32_t rcid);

static bool ch_is_fully_opened(struct channel_ctx *ctx);

struct glink_core_tx_pkt *ch_get_tx_pending_remote_done(struct channel_ctx *ctx,
							uint32_t riid);

static void ch_remove_tx_pending_remote_done(struct channel_ctx *ctx,
					struct glink_core_tx_pkt *tx_pkt);

static void glink_core_rx_cmd_rx_intent_req_ack(struct glink_transport_if
					*if_ptr, uint32_t rcid, bool granted);

static void glink_core_simulate_remote_close_for_xprt(
		struct glink_core_xprt_ctx *xprt_ctx);

static void glink_core_remote_close_common(struct channel_ctx *ctx);

static void check_link_notifier_and_notify(struct glink_core_xprt_ctx *xprt_ptr,
					   enum glink_link_state link_state);

/**
 * glink_ssr() - Clean up locally for SSR by simulating remote close
 * @subsystem:	The name of the subsystem being restarted
 *
 * Call into the transport using the ssr(if_ptr) function to allow it to
 * clean up any necessary structures, then simulate a remote close from
 * subsystem for all channels on that edge.
 *
 * Return: Standard error codes.
 */
int glink_ssr(const char *subsystem)
{
	int ret = 0;
	bool transport_found = false;
	struct glink_core_xprt_ctx *xprt_ctx = NULL;


	mutex_lock(&transport_list_lock_lha0);
	list_for_each_entry(xprt_ctx, &transport_list, list_node) {
		if (!strcmp(subsystem, xprt_ctx->edge) &&
				xprt_is_fully_opened(xprt_ctx)) {
			mutex_unlock(&transport_list_lock_lha0);

			GLINK_INFO_XPRT(xprt_ctx, "%s: SSR\n", __func__);
			xprt_ctx->ops->ssr(xprt_ctx->ops);
			check_link_notifier_and_notify(xprt_ctx,
					GLINK_LINK_STATE_DOWN);
			glink_core_simulate_remote_close_for_xprt(xprt_ctx);
			transport_found = true;
		}
	}
	mutex_unlock(&transport_list_lock_lha0);

	if (!transport_found)
		ret = -ENODEV;

	return ret;
}
EXPORT_SYMBOL(glink_ssr);

/**
 * glink_core_remote_close_common() - Handles the common operations during
 *                                    a remote close.
 * @ctx:	Pointer to channel instance.
 */
static void glink_core_remote_close_common(struct channel_ctx *ctx)
{
	ctx->remote_opened = false;
	ctx->rcid = 0;

	if (ctx->local_open_state != GLINK_CHANNEL_CLOSED) {
		ctx->notify_state(ctx, ctx->user_priv,
				GLINK_REMOTE_DISCONNECTED);
		GLINK_INFO_CH(ctx,
				"%s: %s: GLINK_REMOTE_DISCONNECTED\n",
				__func__, "notify state");
	}

	if (ctx->local_open_state == GLINK_CHANNEL_CLOSED)
		GLINK_INFO_CH(ctx,
			"%s: %s, %s\n", __func__,
			"Did not send GLINK_REMOTE_DISCONNECTED",
			"local state is already CLOSED");

	ch_purge_intent_lists(ctx);
}

/**
 * glink_core_simulate_remote_close_for_xprt() - In the case of SSR,
 *                                               simulate receiving a remote
 *                                               close for all channels on a
 *                                               given transport.
 * xprt_ctx:	Pointer to transport instance
 */
static void glink_core_simulate_remote_close_for_xprt(
		struct glink_core_xprt_ctx *xprt_ctx)
{
	struct channel_ctx *entry;
	unsigned long flags;

	spin_lock_irqsave(&xprt_ctx->xprt_ctx_lock_lhb1, flags);
	list_for_each_entry(entry, &xprt_ctx->channels, port_list_node) {
		GLINK_INFO_CH(entry, "%s: Simulating remote close\n", __func__);
		/*
		 * NOTE:  unlocking here is fine as long as channels are not
		 * removed from the transport.
		 */
		spin_unlock_irqrestore(&xprt_ctx->xprt_ctx_lock_lhb1, flags);
		glink_core_remote_close_common(entry);
		spin_lock_irqsave(&xprt_ctx->xprt_ctx_lock_lhb1, flags);

		GLINK_INFO_CH(entry, "%s: Done simulating remote close\n",
				__func__);
	}
	spin_unlock_irqrestore(&xprt_ctx->xprt_ctx_lock_lhb1, flags);
}

/**
 * tx_linear_vbuf_provider() - Virtual Buffer Provider for linear buffers
 * @iovec:	Pointer to the beginning of the linear buffer.
 * @offset:	Offset into the buffer whose address is needed.
 * @size:	Pointer to hold the length of the contiguous buffer space.
 *
 * This function is used when a linear buffer is transmitted.
 *
 * Return: Address of the buffer which is at offset "offset" from the beginning
 *         of the buffer.
 */
static void *tx_linear_vbuf_provider(void *iovec, size_t offset, size_t *size)
{
	struct glink_core_tx_pkt *tx_info = (struct glink_core_tx_pkt *)iovec;

	if (unlikely(!iovec || !size))
		return NULL;

	if (offset >= tx_info->size)
		return NULL;

	if (unlikely(OVERFLOW_ADD_UNSIGNED(void *, tx_info->data, offset)))
		return NULL;

	*size = tx_info->size - offset;

	return (void *)tx_info->data + offset;
}

/**
 * linearize_vector() - Linearize the vector buffer
 * @iovec:	Pointer to the vector buffer.
 * @size:	Size of data in the vector buffer.
 * vbuf_provider:	Virtual address-space Buffer Provider for the vector.
 * pbuf_provider:	Physical address-space Buffer Provider for the vector.
 *
 * This function is used to linearize the vector buffer provided by the
 * transport when the client has registered to receive only the vector
 * buffer.
 *
 * Return: address of the linear buffer on success, NULL on failure.
 */
static void *linearize_vector(void *iovec, size_t size,
	void * (*vbuf_provider)(void *iovec, size_t offset, size_t *buf_size),
	void * (*pbuf_provider)(void *iovec, size_t offset, size_t *buf_size))
{
	void *bounce_buf;
	void *pdata;
	void *vdata;
	size_t data_size;
	size_t offset = 0;

	bounce_buf = kmalloc(size, GFP_KERNEL);
	if (!bounce_buf)
		return ERR_PTR(-ENOMEM);

	do {
		if (vbuf_provider) {
			vdata = vbuf_provider(iovec, offset, &data_size);
		} else {
			pdata = pbuf_provider(iovec, offset, &data_size);
			vdata = phys_to_virt((unsigned long)pdata);
		}

		if (!vdata)
			break;

		if (OVERFLOW_ADD_UNSIGNED(size_t, data_size, offset)) {
			GLINK_ERR("%s: overflow data_size %zu + offset %zu\n",
				  __func__, data_size, offset);
			goto err;
		}

		memcpy(bounce_buf + offset, vdata, data_size);
		offset += data_size;
	} while (offset < size);

	if (offset != size) {
		GLINK_ERR("%s: Error size_copied %zu != total_size %zu\n",
			  __func__, offset, size);
		goto err;
	}
	return bounce_buf;

err:
	kfree(bounce_buf);
	return NULL;
}

/**
 * xprt_lcid_to_ch_ctx() - lookup a channel by local id
 * @xprt_ctx:	Transport to search for a matching channel.
 * @lcid:	Local channel identifier corresponding to the desired cahnnel.
 *
 * Return: The channel corresponding to @lcid or NULL if a matching channel
 *	is not found.
 */
static struct channel_ctx *xprt_lcid_to_ch_ctx(
					struct glink_core_xprt_ctx *xprt_ctx,
					uint32_t lcid)
{
	struct channel_ctx *entry;
	unsigned long flags;

	spin_lock_irqsave(&xprt_ctx->xprt_ctx_lock_lhb1, flags);
	list_for_each_entry(entry, &xprt_ctx->channels, port_list_node)
		if (entry->lcid == lcid) {
			spin_unlock_irqrestore(&xprt_ctx->xprt_ctx_lock_lhb1,
					flags);
			return entry;
		}
	spin_unlock_irqrestore(&xprt_ctx->xprt_ctx_lock_lhb1, flags);

	return NULL;
}

/**
 * xprt_rcid_to_ch_ctx() - lookup a channel by remote id
 * @xprt_ctx:	Transport to search for a matching channel.
 * @rcid:	Remote channel identifier corresponding to the desired cahnnel.
 *
 * Return: The channel corresponding to @rcid or NULL if a matching channel
 *	is not found.
 */
static struct channel_ctx *xprt_rcid_to_ch_ctx(
					struct glink_core_xprt_ctx *xprt_ctx,
					uint32_t rcid)
{
	struct channel_ctx *entry;
	unsigned long flags;

	spin_lock_irqsave(&xprt_ctx->xprt_ctx_lock_lhb1, flags);
	list_for_each_entry(entry, &xprt_ctx->channels, port_list_node)
		if (entry->rcid == rcid) {
			spin_unlock_irqrestore(&xprt_ctx->xprt_ctx_lock_lhb1,
					flags);
			return entry;
		}
	spin_unlock_irqrestore(&xprt_ctx->xprt_ctx_lock_lhb1, flags);

	return NULL;
}

/**
 * ch_check_duplicate_riid() - Checks for duplicate riid
 * @ctx:	Local channel context
 * @riid:	Remote intent ID
 *
 * This functions check the riid is present in the remote_rx_list or not
 */
bool ch_check_duplicate_riid(struct channel_ctx *ctx, int riid)
{
	struct glink_core_rx_intent *intent;
	unsigned long flags;

	spin_lock_irqsave(&ctx->rmt_rx_intent_lst_lock_lhc2, flags);
	list_for_each_entry(intent, &ctx->rmt_rx_intent_list, list) {
		if (riid == intent->id) {
			spin_unlock_irqrestore(
				&ctx->rmt_rx_intent_lst_lock_lhc2, flags);
			return true;
		}
	}
	spin_unlock_irqrestore(&ctx->rmt_rx_intent_lst_lock_lhc2, flags);
	return false;
}

/**
 * ch_pop_remote_rx_intent() - Finds a matching RX intent
 * @ctx:	Local channel context
 * @size:	Size of Intent
 * @riid_ptr:	Pointer to return value of remote intent ID
 *
 * This functions searches for an RX intent that is >= to the requested size.
 */
int ch_pop_remote_rx_intent(struct channel_ctx *ctx, size_t size,
		uint32_t *riid_ptr)
{
	struct glink_core_rx_intent *intent;
	struct glink_core_rx_intent *intent_tmp;
	unsigned long flags;

	if (GLINK_MAX_PKT_SIZE < size) {
		GLINK_ERR_CH(ctx, "%s: R[]:%zu Invalid size.\n", __func__,
				size);
		return -EINVAL;
	}

	if (riid_ptr == NULL)
		return -EINVAL;

	*riid_ptr = 0;
	spin_lock_irqsave(&ctx->rmt_rx_intent_lst_lock_lhc2, flags);
	if (ctx->transport_ptr->capabilities & GCAP_INTENTLESS) {
		*riid_ptr = ++ctx->dummy_riid;
		spin_unlock_irqrestore(&ctx->rmt_rx_intent_lst_lock_lhc2,
					flags);
		return 0;
	}
	list_for_each_entry_safe(intent, intent_tmp, &ctx->rmt_rx_intent_list,
			list) {
		if (intent->intent_size >= size) {
			list_del(&intent->list);
			*riid_ptr = intent->id;
			kfree(intent);
			spin_unlock_irqrestore(
				&ctx->rmt_rx_intent_lst_lock_lhc2, flags);
			GLINK_DBG_CH(ctx,
					"%s: R[%u]:%zu Removed remote intent\n",
					__func__,
					intent->id,
					intent->intent_size);
			return 0;
		}
	}
	spin_unlock_irqrestore(&ctx->rmt_rx_intent_lst_lock_lhc2, flags);
	return -EAGAIN;
}

/**
 * ch_push_remote_rx_intent() - Registers a remote RX intent
 * @ctx:	Local channel context
 * @size:	Size of Intent
 * @riid:	Remote intent ID
 *
 * This functions adds a remote RX intent to the remote RX intent list.
 */
void ch_push_remote_rx_intent(struct channel_ctx *ctx, size_t size,
		uint32_t riid)
{
	struct glink_core_rx_intent *intent;
	unsigned long flags;

	if (GLINK_MAX_PKT_SIZE < size) {
		GLINK_ERR_CH(ctx, "%s: R[%u]:%zu Invalid size.\n", __func__,
				riid, size);
		return;
	}

	if (ch_check_duplicate_riid(ctx, riid)) {
		GLINK_ERR_CH(ctx, "%s: R[%d]:%zu Duplicate RIID found\n",
				__func__, riid, size);
		return;
	}

	intent = kzalloc(sizeof(struct glink_core_rx_intent), GFP_KERNEL);
	if (!intent) {
		GLINK_ERR_CH(ctx,
			"%s: R[%u]:%zu Memory allocation for intent failed\n",
			__func__, riid, size);
		return;
	}
	intent->id = riid;
	intent->intent_size = size;

	spin_lock_irqsave(&ctx->rmt_rx_intent_lst_lock_lhc2, flags);
	list_add_tail(&intent->list, &ctx->rmt_rx_intent_list);
	spin_unlock_irqrestore(&ctx->rmt_rx_intent_lst_lock_lhc2, flags);

	GLINK_DBG_CH(ctx, "%s: R[%u]:%zu Pushed remote intent\n", __func__,
			intent->id,
			intent->intent_size);
}

/**
 * ch_push_local_rx_intent() - Create an rx_intent
 * @ctx:	Local channel context
 * @pkt_priv:	Data packet
 * @size:	Size of intent
 *
 * This functions creates a local intent and adds it to the local
 * intent list.
 */
struct glink_core_rx_intent *ch_push_local_rx_intent(struct channel_ctx *ctx,
		const void *pkt_priv, size_t size)
{
	struct glink_core_rx_intent *intent;
	unsigned long flags;
	int ret;

	if (GLINK_MAX_PKT_SIZE < size) {
		GLINK_ERR_CH(ctx,
			"%s: L[]:%zu Invalid size\n", __func__, size);
		return NULL;
	}

	if (!pkt_priv) {
		GLINK_ERR_CH(ctx, "%s: Invalid packet", __func__);
		return NULL;
	}

	intent = ch_get_free_local_rx_intent(ctx);
	if (!intent) {
		if (ctx->max_used_liid >= ctx->transport_ptr->max_iid) {
			GLINK_ERR_CH(ctx,
				"%s: All intents are in USE max_iid[%d]",
				__func__, ctx->transport_ptr->max_iid);
			return NULL;
		}

		intent = kzalloc(sizeof(struct glink_core_rx_intent),
								GFP_KERNEL);
		if (!intent) {
			GLINK_ERR_CH(ctx,
			"%s: Memory Allocation for local rx_intent failed",
				__func__);
			return NULL;
		}
		intent->id = ++ctx->max_used_liid;
	}

	/* transport is responsible for allocating/reserving for the intent */
	ret = ctx->transport_ptr->ops->allocate_rx_intent(size, intent);
	if (ret < 0) {
		/* intent data allocation failure */
		GLINK_ERR_CH(ctx, "%s: unable to allocate intent sz[%zu] %d",
			__func__, size, ret);
		spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
		list_add_tail(&intent->list,
				&ctx->local_rx_intent_free_list);
		spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1,
				flags);
		return NULL;
	}

	intent->pkt_priv = pkt_priv;
	intent->intent_size = size;
	intent->write_offset = 0;
	intent->pkt_size = 0;
	intent->bounce_buf = NULL;

	spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	list_add_tail(&intent->list, &ctx->local_rx_intent_list);
	spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	GLINK_DBG_CH(ctx, "%s: L[%u]:%zu Pushed intent\n", __func__,
			intent->id,
			intent->intent_size);
	return intent;
}

/**
 * ch_remove_local_rx_intent() - Find and remove RX Intent from list
 * @ctx:	Local channel context
 * @liid:	Local channel Intent ID
 *
 * This functions parses the local intent list for a specific channel
 * and checks for the intent using the intent ID. If found, the intent
 * is deleted from the list.
 */
void ch_remove_local_rx_intent(struct channel_ctx *ctx, uint32_t liid)
{
	struct glink_core_rx_intent *intent, *tmp_intent;
	unsigned long flags;

	if (ctx->transport_ptr->max_iid < liid) {
		GLINK_ERR_CH(ctx, "%s: L[%u] Invalid ID.\n", __func__,
				liid);
		return;
	}

	spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	list_for_each_entry_safe(intent, tmp_intent, &ctx->local_rx_intent_list,
									list) {
		if (liid == intent->id) {
			list_del(&intent->list);
			list_add_tail(&intent->list,
					&ctx->local_rx_intent_free_list);
			spin_unlock_irqrestore(
					&ctx->local_rx_intent_lst_lock_lhc1,
					flags);
			GLINK_DBG_CH(ctx,
			"%s: L[%u]:%zu moved intent to Free/unused list\n",
				__func__,
				intent->id,
				intent->intent_size);
			return;
		}
	}
	spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	GLINK_ERR_CH(ctx, "%s: L[%u] Intent not found.\n", __func__,
			liid);
}

/**
 * ch_get_dummy_rx_intent() - Get a dummy rx_intent
 * @ctx:	Local channel context
 * @liid:	Local channel Intent ID
 *
 * This functions parses the local intent list for a specific channel and
 * returns either a matching intent or allocates a dummy one if no matching
 * intents can be found.
 *
 * Return: Pointer to the intent if intent is found else NULL
 */
struct glink_core_rx_intent *ch_get_dummy_rx_intent(struct channel_ctx *ctx,
		uint32_t liid)
{
	struct glink_core_rx_intent *intent;
	unsigned long flags;

	spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	if (!list_empty(&ctx->local_rx_intent_list)) {
		intent = list_first_entry(&ctx->local_rx_intent_list,
					  struct glink_core_rx_intent, list);
		spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1,
					flags);
		return intent;
	}
	spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1, flags);

	intent = ch_get_free_local_rx_intent(ctx);
	if (!intent) {
		intent = kzalloc(sizeof(struct glink_core_rx_intent),
								GFP_KERNEL);
		if (!intent) {
			GLINK_ERR_CH(ctx,
			"%s: Memory Allocation for local rx_intent failed",
				__func__);
			return NULL;
		}
		intent->id = ++ctx->max_used_liid;
	}
	intent->intent_size = 0;
	intent->write_offset = 0;
	intent->pkt_size = 0;
	intent->bounce_buf = NULL;
	intent->pkt_priv = NULL;

	spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	list_add_tail(&intent->list, &ctx->local_rx_intent_list);
	spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	GLINK_DBG_CH(ctx, "%s: L[%u]:%zu Pushed intent\n", __func__,
			intent->id,
			intent->intent_size);
	return intent;
}

/**
 * ch_get_local_rx_intent() - Search for an rx_intent
 * @ctx:	Local channel context
 * @liid:	Local channel Intent ID
 *
 * This functions parses the local intent list for a specific channel
 * and checks for the intent using the intent ID. If found, pointer to
 * the intent is returned.
 *
 * Return: Pointer to the intent if intent is found else NULL
 */
struct glink_core_rx_intent *ch_get_local_rx_intent(struct channel_ctx *ctx,
		uint32_t liid)
{
	struct glink_core_rx_intent *intent;
	unsigned long flags;

	if (ctx->transport_ptr->max_iid < liid) {
		GLINK_ERR_CH(ctx, "%s: L[%u] Invalid ID.\n", __func__,
				liid);
		return NULL;
	}

	if (ctx->transport_ptr->capabilities & GCAP_INTENTLESS)
		return ch_get_dummy_rx_intent(ctx, liid);

	spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	list_for_each_entry(intent, &ctx->local_rx_intent_list, list) {
		if (liid == intent->id) {
			spin_unlock_irqrestore(
				&ctx->local_rx_intent_lst_lock_lhc1, flags);
			return intent;
		}
	}
	spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	GLINK_ERR_CH(ctx, "%s: L[%u] Intent not found.\n", __func__,
			liid);
	return NULL;
}

/**
 * ch_set_local_rx_intent_notified() - Add a rx intent to local intent
 *					notified list
 * @ctx:	Local channel context
 * @intent_ptr:	Pointer to the local intent
 *
 * This functions parses the local intent list for a specific channel
 * and checks for the intent. If found, the function deletes the intent
 * from local_rx_intent list and adds it to local_rx_intent_notified list.
 */
void ch_set_local_rx_intent_notified(struct channel_ctx *ctx,
		struct glink_core_rx_intent *intent_ptr)
{
	struct glink_core_rx_intent *tmp_intent, *intent;
	unsigned long flags;

	spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	list_for_each_entry_safe(intent, tmp_intent, &ctx->local_rx_intent_list,
									list) {
		if (intent == intent_ptr) {
			list_del(&intent->list);
			list_add_tail(&intent->list,
				&ctx->local_rx_intent_ntfy_list);
			GLINK_DBG_CH(ctx,
				"%s: L[%u]:%zu Moved intent %s",
				__func__,
				intent_ptr->id,
				intent_ptr->intent_size,
				"from local to notify list\n");
			spin_unlock_irqrestore(
					&ctx->local_rx_intent_lst_lock_lhc1,
					flags);
			return;
		}
	}
	spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	GLINK_ERR_CH(ctx, "%s: L[%u] Intent not found.\n", __func__,
			intent_ptr->id);
}

/**
 * ch_get_local_rx_intent_notified() - Find rx intent in local notified list
 * @ctx:	Local channel context
 * @ptr:	Pointer to the rx intent
 *
 * This functions parses the local intent notify list for a specific channel
 * and checks for the intent.
 *
 * Return: Pointer to the intent if intent is found else NULL.
 */
struct glink_core_rx_intent *ch_get_local_rx_intent_notified(
	struct channel_ctx *ctx, const void *ptr)
{
	struct glink_core_rx_intent *ptr_intent;
	unsigned long flags;

	spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	list_for_each_entry(ptr_intent, &ctx->local_rx_intent_ntfy_list,
								list) {
		if (ptr_intent->data == ptr || ptr_intent->iovec == ptr ||
		    ptr_intent->bounce_buf == ptr) {
			spin_unlock_irqrestore(
					&ctx->local_rx_intent_lst_lock_lhc1,
					flags);
			return ptr_intent;
		}
	}
	spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	GLINK_ERR_CH(ctx, "%s: Local intent not found\n", __func__);
	return NULL;
}

/**
 * ch_remove_local_rx_intent_notified() - Remove a rx intent in local intent
 *					notified list
 * @ctx:	Local channel context
 * @ptr:	Pointer to the rx intent
 *
 * This functions parses the local intent notify list for a specific channel
 * and checks for the intent. If found, the function deletes the intent
 * from local_rx_intent_notified list and adds it to local_rx_intent_free list.
 */
void ch_remove_local_rx_intent_notified(struct channel_ctx *ctx,
	struct glink_core_rx_intent *liid_ptr)
{
	struct glink_core_rx_intent *ptr_intent, *tmp_intent;
	unsigned long flags;

	spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	list_for_each_entry_safe(ptr_intent, tmp_intent,
				&ctx->local_rx_intent_ntfy_list, list) {
		if (ptr_intent == liid_ptr) {
			list_del(&ptr_intent->list);
			GLINK_DBG_CH(ctx,
				"%s: L[%u]:%zu Removed intent from notify list\n",
				__func__,
				ptr_intent->id,
				ptr_intent->intent_size);
			kfree(ptr_intent->bounce_buf);
			ptr_intent->bounce_buf = NULL;
			list_add_tail(&ptr_intent->list,
				&ctx->local_rx_intent_free_list);
			spin_unlock_irqrestore(
					&ctx->local_rx_intent_lst_lock_lhc1,
					flags);
			return;
		}
	}
	spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	GLINK_ERR_CH(ctx, "%s: L[%u] Intent not found.\n", __func__,
			liid_ptr->id);
}

/**
 * ch_get_free_local_rx_intent() - Return a rx intent in local intent
 *					free list
 * @ctx:	Local channel context
 *
 * This functions parses the local_rx_intent_free list for a specific channel
 * and checks for the free unused intent. If found, the function returns
 * the free intent pointer else NULL pointer.
 */
struct glink_core_rx_intent *ch_get_free_local_rx_intent(
	struct channel_ctx *ctx)
{
	struct glink_core_rx_intent *ptr_intent = NULL;
	unsigned long flags;

	spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	if (!list_empty(&ctx->local_rx_intent_free_list)) {
		ptr_intent = list_first_entry(&ctx->local_rx_intent_free_list,
				struct glink_core_rx_intent,
				list);
		list_del(&ptr_intent->list);
	}
	spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	return ptr_intent;
}

/**
 * ch_purge_intent_lists() - Remove all intents for a channel
 *
 * @ctx:	Local channel context
 *
 * This functions parses the local intent lists for a specific channel and
 * removes and frees all intents.
 */
void ch_purge_intent_lists(struct channel_ctx *ctx)
{
	struct glink_core_rx_intent *ptr_intent, *tmp_intent;
	struct glink_core_tx_pkt *tx_info, *tx_info_temp;
	unsigned long flags;

	mutex_lock(&ctx->tx_lists_mutex_lhc3);
	list_for_each_entry_safe(tx_info, tx_info_temp, &ctx->tx_active,
			list_node) {
		ctx->notify_tx_abort(ctx, ctx->user_priv,
				tx_info->pkt_priv);
		list_del(&tx_info->list_node);
		kfree(tx_info);
	}
	mutex_unlock(&ctx->tx_lists_mutex_lhc3);

	spin_lock_irqsave(&ctx->local_rx_intent_lst_lock_lhc1, flags);
	list_for_each_entry_safe(ptr_intent, tmp_intent,
				&ctx->local_rx_intent_list, list) {
		ctx->notify_rx_abort(ctx, ctx->user_priv,
				ptr_intent->pkt_priv);
		list_del(&ptr_intent->list);
		kfree(ptr_intent);
	}

	if (!list_empty(&ctx->local_rx_intent_ntfy_list))
		/*
		 * The client is still processing an rx_notify() call and has
		 * not yet called glink_rx_done() to return the pointer to us.
		 * glink_rx_done() will do the appropriate cleanup when this
		 * call occurs, but log a message here just for internal state
		 * tracking.
		 */
		GLINK_INFO_CH(ctx, "%s: waiting on glink_rx_done()\n",
				__func__);

	list_for_each_entry_safe(ptr_intent, tmp_intent,
				&ctx->local_rx_intent_free_list, list) {
		list_del(&ptr_intent->list);
		kfree(ptr_intent);
	}
	ctx->max_used_liid = 0;
	spin_unlock_irqrestore(&ctx->local_rx_intent_lst_lock_lhc1, flags);

	spin_lock_irqsave(&ctx->rmt_rx_intent_lst_lock_lhc2, flags);
	list_for_each_entry_safe(ptr_intent, tmp_intent,
			&ctx->rmt_rx_intent_list, list) {
		list_del(&ptr_intent->list);
		kfree(ptr_intent);
	}
	spin_unlock_irqrestore(&ctx->rmt_rx_intent_lst_lock_lhc2, flags);
}

/**
 * ch_get_tx_pending_remote_done() - Lookup for a packet that is waiting for
 *                                   the remote-done notification.
 * @ctx:	Pointer to the channel context
 * @riid:	riid of transmit packet
 *
 * This function adds a packet to the tx_pending_remote_done list.
 *
 * The tx_lists_mutex_lhc3 lock needs to be held while calling this function.
 *
 * Return: Pointer to the tx packet
 */
struct glink_core_tx_pkt *ch_get_tx_pending_remote_done(
	struct channel_ctx *ctx, uint32_t riid)
{
	struct glink_core_tx_pkt *tx_pkt;

	if (!ctx) {
		GLINK_ERR("%s: Invalid context pointer", __func__);
		return NULL;
	}

	list_for_each_entry(tx_pkt, &ctx->tx_pending_remote_done, list_node) {
		if (tx_pkt->riid == riid)
			return tx_pkt;
	}

	GLINK_ERR_CH(ctx, "%s: R[%u] Tx packet for intent not found.\n",
			__func__, riid);
	return NULL;
}

/**
 * ch_remove_tx_pending_remote_done() - Removes a packet transmit context for a
 *                     packet that is waiting for the remote-done notification
 * @ctx:	Pointer to the channel context
 * @tx_pkt:	Pointer to the transmit packet
 *
 * This function parses through tx_pending_remote_done and removes a
 * packet that matches with the tx_pkt.
 */
void ch_remove_tx_pending_remote_done(struct channel_ctx *ctx,
	struct glink_core_tx_pkt *tx_pkt)
{
	struct glink_core_tx_pkt *local_tx_pkt, *tmp_tx_pkt;

	if (!ctx || !tx_pkt) {
		GLINK_ERR("%s: Invalid input", __func__);
		return;
	}

	list_for_each_entry_safe(local_tx_pkt, tmp_tx_pkt,
			&ctx->tx_pending_remote_done, list_node) {
		if (tx_pkt == local_tx_pkt) {
			list_del(&local_tx_pkt->list_node);
			GLINK_DBG_CH(ctx,
				"%s: R[%u] Removed Tx packet for intent\n",
				__func__,
				tx_pkt->riid);
			kfree(local_tx_pkt);
			return;
		}
	}

	GLINK_ERR_CH(ctx, "%s: R[%u] Tx packet for intent not found", __func__,
			tx_pkt->riid);
}

/**
 * ch_name_to_ch_ctx_create() - lookup a channel by name, create the channel if
 *                              it is not found.
 * @xprt_ctx:	Transport to search for a matching channel.
 * @name:	Name of the desired channel.
 *
 * Return: The channel corresponding to @name, NULL if a matching channel was
 *         not found AND a new channel could not be created.
 */
static struct channel_ctx *ch_name_to_ch_ctx_create(
					struct glink_core_xprt_ctx *xprt_ctx,
					const char *name)
{
	struct channel_ctx *entry;
	struct channel_ctx *ctx;
	struct channel_ctx *temp;
	unsigned long flags;

	ctx = kzalloc(sizeof(struct channel_ctx), GFP_KERNEL);
	if (!ctx) {
		GLINK_ERR_XPRT(xprt_ctx, "%s: Failed to allocated ctx, %s",
			"checking if there is one existing\n",
			__func__);
		goto check_ctx;
	}

	ctx->local_open_state = GLINK_CHANNEL_CLOSED;
	strlcpy(ctx->name, name, GLINK_NAME_SIZE);
	INIT_LIST_HEAD(&ctx->tx_ready_list_node);
	init_completion(&ctx->int_req_ack_complete);
	init_completion(&ctx->int_req_complete);
	INIT_LIST_HEAD(&ctx->local_rx_intent_list);
	INIT_LIST_HEAD(&ctx->local_rx_intent_ntfy_list);
	INIT_LIST_HEAD(&ctx->local_rx_intent_free_list);
	spin_lock_init(&ctx->local_rx_intent_lst_lock_lhc1);
	INIT_LIST_HEAD(&ctx->rmt_rx_intent_list);
	spin_lock_init(&ctx->rmt_rx_intent_lst_lock_lhc2);
	INIT_LIST_HEAD(&ctx->tx_active);
	INIT_LIST_HEAD(&ctx->tx_pending_remote_done);
	mutex_init(&ctx->tx_lists_mutex_lhc3);

check_ctx:
	spin_lock_irqsave(&xprt_ctx->xprt_ctx_lock_lhb1, flags);
	list_for_each_entry_safe(entry, temp, &xprt_ctx->channels,
		    port_list_node)
		if (!strcmp(entry->name, name)) {
			spin_unlock_irqrestore(&xprt_ctx->xprt_ctx_lock_lhb1,
					flags);
			kfree(ctx);

			return entry;
		}

	if (ctx) {
		if (xprt_ctx->next_lcid > xprt_ctx->max_cid) {
			/* no more channels available */
			GLINK_ERR_XPRT(xprt_ctx,
				"%s: unable to exceed %u channels\n",
				__func__, xprt_ctx->max_cid);
			spin_unlock_irqrestore(&xprt_ctx->xprt_ctx_lock_lhb1,
					flags);
			kfree(ctx);
			return NULL;
		}
		ctx->lcid = xprt_ctx->next_lcid++;
		list_add_tail(&ctx->port_list_node, &xprt_ctx->channels);

		GLINK_INFO_PERF_CH_XPRT(ctx, xprt_ctx,
			"%s: local:GLINK_CHANNEL_CLOSED\n",
			__func__);
	}
	spin_unlock_irqrestore(&xprt_ctx->xprt_ctx_lock_lhb1, flags);
	if (ctx != NULL)
		glink_debugfs_add_channel(ctx, xprt_ctx);
	return ctx;
}

/**
 * ch_add_rcid() - add a remote channel identifier to an existing channel
 * @xprt_ctx:	Transport the channel resides on.
 * @ctx:	Channel receiving the identifier.
 * @rcid:	The remote channel identifier.
 */
static void ch_add_rcid(struct glink_core_xprt_ctx *xprt_ctx,
			struct channel_ctx *ctx,
			uint32_t rcid)
{
	ctx->rcid = rcid;
}

/*
 * ch_is_fully_opened() - Verify if a channel is open
 * ctx:	Pointer to channel context
 *
 * Return: True if open, else flase
 */
static bool ch_is_fully_opened(struct channel_ctx *ctx)
{
	if (ctx->remote_opened && ctx->local_open_state == GLINK_CHANNEL_OPENED)
		return true;

	return false;
}

/**
 * find_open_transport() - find a specific open transport
 * @edge:	Edge the transport is on.
 * @name:	Name of the transport (or NULL if no preference)
 *
 * Find an open transport corresponding to the specified @name and @edge.  @edge
 * is expected to be valid.  @name is expected to be NULL (unspecified) or
 * valid.  If @name is not specified, then the first transport found on the
 * specified edge will be returned.
 *
 * Return: Transport with the specified name on the specified edge, if open.
 *	NULL if the transport exists, but is not fully open.  ENODEV if no such
 *	transport exists.
 */
static struct glink_core_xprt_ctx *find_open_transport(const char *edge,
						       const char *name)
{
	struct glink_core_xprt_ctx *xprt;
	struct glink_core_xprt_ctx *temp;
	struct glink_core_xprt_ctx *ret;

	ret = (struct glink_core_xprt_ctx *)ERR_PTR(-ENODEV);

	mutex_lock(&transport_list_lock_lha0);
	list_for_each_entry_safe(xprt, temp, &transport_list, list_node)
		if (!strcmp(edge, xprt->edge)) {
			if (name) {
				if (!strcmp(name, xprt->name) &&
						xprt_is_fully_opened(xprt)) {
					mutex_unlock(&transport_list_lock_lha0);
					return xprt;
				}
			} else if (xprt_is_fully_opened(xprt)) {
					mutex_unlock(&transport_list_lock_lha0);
					return xprt;
			}
			ret = NULL;
		}

	mutex_unlock(&transport_list_lock_lha0);
	return ret;
}

/**
 * xprt_is_fully_opened() - check the open status of a transport
 * @xprt:	Transport being checked.
 *
 * Return: True if the transport is fully opened, false otherwise.
 */
static bool xprt_is_fully_opened(struct glink_core_xprt_ctx *xprt)
{
	if (xprt->remote_neg_completed &&
					xprt->local_state == GLINK_XPRT_OPENED)
		return true;

	return false;
}

/**
 * glink_dummy_notify_rx_intent_req() - Dummy RX Request
 *
 * @handle:	Channel handle (ignored)
 * @priv:	Private data pointer (ignored)
 * @req_size:	Requested size (ignored)
 *
 * Dummy RX intent request if client does not implement the optional callback
 * function.
 *
 * Return:  False
 */
static bool glink_dummy_notify_rx_intent_req(void *handle, const void *priv,
	size_t req_size)
{
	return false;
}

/**
 * glink_dummy_notify_rx_sigs() - Dummy signal callback
 *
 * @handle:	Channel handle (ignored)
 * @priv:	Private data pointer (ignored)
 * @req_size:	Requested size (ignored)
 *
 * Dummy signal callback if client does not implement the optional callback
 * function.
 *
 * Return:  False
 */
static void glink_dummy_notify_rx_sigs(void *handle, const void *priv,
				uint32_t old_sigs, uint32_t new_sigs)
{
	/* intentionally left blank */
}

/**
 * glink_dummy_rx_abort() - Dummy rx abort callback
 *
 * handle:	Channel handle (ignored)
 * priv:	Private data pointer (ignored)
 * pkt_priv:	Private intent data pointer (ignored)
 *
 * Dummy rx abort callback if client does not implement the optional callback
 * function.
 */
static void glink_dummy_notify_rx_abort(void *handle, const void *priv,
				const void *pkt_priv)
{
	/* intentionally left blank */
}

/**
 * glink_dummy_tx_abort() - Dummy tx abort callback
 *
 * @handle:	Channel handle (ignored)
 * @priv:	Private data pointer (ignored)
 * @pkt_priv:	Private intent data pointer (ignored)
 *
 * Dummy tx abort callback if client does not implement the optional callback
 * function.
 */
static void glink_dummy_notify_tx_abort(void *handle, const void *priv,
				const void *pkt_priv)
{
	/* intentionally left blank */
}

/**
 * dummy_poll() - a dummy poll() for transports that don't define one
 * @if_ptr:	The transport interface handle for this transport.
 * @lcid:	The channel to poll.
 *
 * Return: An error to indicate that this operation is unsupported.
 */
static int dummy_poll(struct glink_transport_if *if_ptr, uint32_t lcid)
{
	return -EOPNOTSUPP;
}

/**
 * dummy_mask_rx_irq() - a dummy mask_rx_irq() for transports that don't define
 *			 one
 * @if_ptr:	The transport interface handle for this transport.
 * @lcid:	The local channel id for this channel.
 * @mask:	True to mask the irq, false to unmask.
 * @pstruct:	Platform defined structure with data necessary for masking.
 *
 * Return: An error to indicate that this operation is unsupported.
 */
static int dummy_mask_rx_irq(struct glink_transport_if *if_ptr, uint32_t lcid,
			     bool mask, void *pstruct)
{
	return -EOPNOTSUPP;
}

/**
 * notif_if_up_all_xprts() - Check and notify existing transport state if up
 * @notif_info:	Data structure containing transport information to be notified.
 *
 * This function is called when the client registers a notifier to know about
 * the state of a transport. This function matches the existing transports with
 * the transport in the "notif_info" parameter. When a matching transport is
 * found, the callback function in the "notif_info" parameter is called with
 * the state of the matching transport.
 *
 * If an edge or transport is not defined, then all edges and/or transports
 * will be matched and will receive up notifications.
 */
static void notif_if_up_all_xprts(
		struct link_state_notifier_info *notif_info)
{
	struct glink_core_xprt_ctx *xprt_ptr;
	struct glink_link_state_cb_info cb_info;

	cb_info.link_state = GLINK_LINK_STATE_UP;
	mutex_lock(&transport_list_lock_lha0);
	list_for_each_entry(xprt_ptr, &transport_list, list_node) {
		if (strlen(notif_info->edge) &&
		    strcmp(notif_info->edge, xprt_ptr->edge))
			continue;

		if (strlen(notif_info->transport) &&
		    strcmp(notif_info->transport, xprt_ptr->name))
			continue;

		if (!xprt_is_fully_opened(xprt_ptr))
			continue;

		cb_info.transport = xprt_ptr->name;
		cb_info.edge = xprt_ptr->edge;
		notif_info->glink_link_state_notif_cb(&cb_info,
						notif_info->priv);
	}
	mutex_unlock(&transport_list_lock_lha0);
}

/**
 * check_link_notifier_and_notify() - Check and notify clients about link state
 * @xprt_ptr:	Transport whose state to be notified.
 * @link_state:	State of the transport to be notified.
 *
 * This function is called when the state of the transport changes. This
 * function matches the transport with the clients that have registered to
 * be notified about the state changes. When a matching client notifier is
 * found, the callback function in the client notifier is called with the
 * new state of the transport.
 */
static void check_link_notifier_and_notify(struct glink_core_xprt_ctx *xprt_ptr,
					   enum glink_link_state link_state)
{
	struct link_state_notifier_info *notif_info;
	struct glink_link_state_cb_info cb_info;

	cb_info.link_state = link_state;
	mutex_lock(&link_state_notifier_lock_lha1);
	list_for_each_entry(notif_info, &link_state_notifier_list, list) {
		if (strlen(notif_info->edge) &&
		    strcmp(notif_info->edge, xprt_ptr->edge))
			continue;

		if (strlen(notif_info->transport) &&
		    strcmp(notif_info->transport, xprt_ptr->name))
			continue;

		cb_info.transport = xprt_ptr->name;
		cb_info.edge = xprt_ptr->edge;
		notif_info->glink_link_state_notif_cb(&cb_info,
						notif_info->priv);
	}
	mutex_unlock(&link_state_notifier_lock_lha1);
}

/**
 * Open GLINK channel.
 *
 * @cfg_ptr:	Open configuration structure (the structure is copied before
 *		glink_open returns).  All unused fields should be zero-filled.
 *
 * Return:  Pointer to channel on success, PTR_ERR() with standard Linux
 * error code on failure.
 */
void *glink_open(const struct glink_open_config *cfg)
{
	struct channel_ctx *ctx = NULL;
	struct glink_core_xprt_ctx *transport_ptr;
	size_t len;
	int ret;

	if (!cfg->edge || !cfg->name) {
		GLINK_ERR("%s: !cfg->edge || !cfg->name\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	len = strlen(cfg->edge);
	if (len == 0 || len >= GLINK_NAME_SIZE) {
		GLINK_ERR("%s: [EDGE] len == 0 || len >= GLINK_NAME_SIZE\n",
				__func__);
		return ERR_PTR(-EINVAL);
	}

	len = strlen(cfg->name);
	if (len == 0 || len >= GLINK_NAME_SIZE) {
		GLINK_ERR("%s: [NAME] len == 0 || len >= GLINK_NAME_SIZE\n",
				__func__);
		return ERR_PTR(-EINVAL);
	}

	if (cfg->transport) {
		len = strlen(cfg->transport);
		if (len == 0 || len >= GLINK_NAME_SIZE) {
			GLINK_ERR("%s: [TRANSPORT] len == 0 || %s\n",
				__func__,
				"len >= GLINK_NAME_SIZE");
			return ERR_PTR(-EINVAL);
		}
	}

	/* confirm required notification parameters */
	if (!(cfg->notify_rx || cfg->notify_rxv) || !cfg->notify_tx_done ||
			!cfg->notify_state || !cfg->notify_rx_intent_req) {
		GLINK_ERR("%s: Incorrect notification parameters\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	/* find transport */
	transport_ptr = find_open_transport(cfg->edge, cfg->transport);
	if (IS_ERR_OR_NULL(transport_ptr)) {
		GLINK_ERR("%s:%s %s: Error %d - unable to find transport\n",
				cfg->transport, cfg->edge, __func__,
				(unsigned)PTR_ERR(transport_ptr));
		return ERR_PTR(-ENODEV);
	}

	/*
	 * look for an existing port structure which can occur in
	 * reopen and remote-open-first cases
	 */
	ctx = ch_name_to_ch_ctx_create(transport_ptr, cfg->name);
	if (ctx == NULL) {
		GLINK_ERR("%s:%s %s: Error - unable to allocate new channel\n",
				cfg->transport, cfg->edge, __func__);
		return ERR_PTR(-ENOMEM);
	}

	/* port already exists */
	if (ctx->local_open_state != GLINK_CHANNEL_CLOSED) {
		/* not ready to be re-opened */
		GLINK_INFO_CH_XPRT(ctx, transport_ptr,
		"%s: Channel not ready to be re-opened. State: %u\n",
		__func__, ctx->local_open_state);
		return ERR_PTR(-EBUSY);
	}

	/* initialize port structure */
	ctx->user_priv = cfg->priv;
	ctx->notify_rx = cfg->notify_rx;
	ctx->notify_tx_done = cfg->notify_tx_done;
	ctx->notify_state = cfg->notify_state;
	ctx->notify_rx_intent_req = cfg->notify_rx_intent_req;
	ctx->notify_rxv = cfg->notify_rxv;
	ctx->notify_rx_sigs = cfg->notify_rx_sigs;
	ctx->notify_rx_abort = cfg->notify_rx_abort;
	ctx->notify_tx_abort = cfg->notify_tx_abort;

	if (!ctx->notify_rx_intent_req)
		ctx->notify_rx_intent_req = glink_dummy_notify_rx_intent_req;
	if (!ctx->notify_rx_sigs)
		ctx->notify_rx_sigs = glink_dummy_notify_rx_sigs;
	if (!ctx->notify_rx_abort)
		ctx->notify_rx_abort = glink_dummy_notify_rx_abort;
	if (!ctx->notify_tx_abort)
		ctx->notify_tx_abort = glink_dummy_notify_tx_abort;

	ctx->transport_ptr = transport_ptr;
	ctx->local_open_state = GLINK_CHANNEL_OPENING;
	GLINK_INFO_PERF_CH(ctx,
		"%s: local:GLINK_CHANNEL_CLOSED->GLINK_CHANNEL_OPENING\n",
		__func__);

	/* start local-open sequence */
	ret = ctx->transport_ptr->ops->tx_cmd_ch_open(ctx->transport_ptr->ops,
		ctx->lcid, cfg->name);
	if (ret) {
		/* failure to send open command (transport failure) */
		ctx->local_open_state = GLINK_CHANNEL_CLOSED;
		GLINK_ERR_CH(ctx, "%s: Unable to send open command %d\n",
			__func__, ret);
		return ERR_PTR(ret);
	}

	GLINK_INFO_CH(ctx, "%s: Created channel, sent OPEN command. %s:%p\n",
		__func__, "Handle address: ", ctx);

	return ctx;
}
EXPORT_SYMBOL(glink_open);

/**
 * glink_get_channel_id_for_handle() - Get logical channel ID
 *
 * @handle:	handle of channel
 *
 * Used internally by G-Link debugfs.
 *
 * Return:  Logical Channel ID or standard Linux error code
 */
int glink_get_channel_id_for_handle(void *handle)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;
	if (ctx == NULL)
		return -EINVAL;

	return ctx->lcid;
}
EXPORT_SYMBOL(glink_get_channel_id_for_handle);

/**
 * glink_get_channel_name_for_handle() - return channel name
 *
 * @handle:	handle of channel
 *
 * Used internally by G-Link debugfs.
 *
 * Return:  Channel name or NULL
 */
char *glink_get_channel_name_for_handle(void *handle)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;
	if (ctx == NULL)
		return NULL;

	return ctx->name;
}
EXPORT_SYMBOL(glink_get_channel_name_for_handle);

/**
 * glink_close() - Close a previously opened channel.
 *
 * @handle:	handle to close
 *
 * Once the closing process has been completed, the GLINK_LOCAL_DISCONNECTED
 * state event will be sent and the channel can be reopened.
 *
 * Return:  0 on success; -EINVAL for invalid handle, -EBUSY is close is
 * already in progress, standard Linux Error code otherwise.
 */
int glink_close(void *handle)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;
	int ret;

	GLINK_INFO_CH(ctx, "%s: Closing channel, handle address: %p\n",
			__func__, ctx);
	if (!ctx)
		return -EINVAL;

	if (ctx->local_open_state == GLINK_CHANNEL_CLOSED)
		return 0;

	if (ctx->local_open_state == GLINK_CHANNEL_CLOSING)
		/* close already pending */
		return -EBUSY;

	mutex_lock(&ctx->transport_ptr->tx_ready_mutex_lhb2);
	if (!list_empty(&ctx->tx_ready_list_node))
		list_del_init(&ctx->tx_ready_list_node);
	mutex_unlock(&ctx->transport_ptr->tx_ready_mutex_lhb2);

	GLINK_INFO_PERF_CH(ctx,
		"%s: local:%u->GLINK_CHANNEL_CLOSING\n",
		__func__, ctx->local_open_state);
	ctx->local_open_state = GLINK_CHANNEL_CLOSING;

	/* send close command */
	ret = ctx->transport_ptr->ops->tx_cmd_ch_close(ctx->transport_ptr->ops,
			ctx->lcid);
	return ret;
}
EXPORT_SYMBOL(glink_close);

/**
 * glink_tx_common() - Common TX implementation
 *
 * @handle:	handle returned by glink_open()
 * @pkt_priv:	opaque data value that will be returned to client with
 *		notify_tx_done notification
 * @data:	pointer to the data
 * @size:	size of data
 * @vbuf_provider: Virtual Address-space Buffer Provider for the tx buffer.
 * @vbuf_provider: Physical Address-space Buffer Provider for the tx buffer.
 * @tx_flags:	Flags to indicate transmit options
 *
 * Return:	-EINVAL for invalid handle; -EBUSY if channel isn't ready for
 *		transmit operation (not fully opened); -EAGAIN if remote side
 *		has not provided a receive intent that is big enough.
 */
static int glink_tx_common(void *handle, void *pkt_priv,
	void *data, void *iovec, size_t size,
	void * (*vbuf_provider)(void *iovec, size_t offset, size_t *size),
	void * (*pbuf_provider)(void *iovec, size_t offset, size_t *size),
	uint32_t tx_flags)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;
	uint32_t riid;
	int ret = 0;
	struct glink_core_tx_pkt *tx_info;

	if (!ctx)
		return -EINVAL;

	if (!(vbuf_provider || pbuf_provider))
		return -EINVAL;

	if (!ch_is_fully_opened(ctx))
		return -EBUSY;

	if (size > GLINK_MAX_PKT_SIZE)
		return -EINVAL;

	/* find matching rx intent (first-fit algorithm for now) */
	if (ch_pop_remote_rx_intent(ctx, size, &riid)) {
		if (!(tx_flags & GLINK_TX_REQ_INTENT)) {
			/* no rx intent available */
			GLINK_ERR_CH(ctx,
				"%s: R[%u]:%zu Intent not present for lcid\n",
				__func__, riid, size);
			return -EAGAIN;
		} else {
			/* request intent of correct size */
			INIT_COMPLETION(ctx->int_req_ack_complete);
			INIT_COMPLETION(ctx->int_req_complete);
			ret = ctx->transport_ptr->ops->tx_cmd_rx_intent_req(
				ctx->transport_ptr->ops, ctx->lcid, size);
			if (ret)
				return ret;

			/* wait for the remote intent req ack */
			wait_for_completion(&ctx->int_req_ack_complete);
			if (!ctx->int_req_ack) {
				GLINK_ERR_CH(ctx,
				    "%s: Intent Request with size: %zu %s",
				    __func__, size,
				    "not granted for lcid\n");
				return -EAGAIN;
			}

			/* wait for the rx_intent from remote side */
			do {
				wait_for_completion(&ctx->int_req_complete);
				INIT_COMPLETION(ctx->int_req_complete);
			} while (ch_pop_remote_rx_intent(ctx, size, &riid));
		}
	}

	tx_info = kzalloc(sizeof(struct glink_core_tx_pkt), GFP_KERNEL);
	if (!tx_info) {
		GLINK_ERR_CH(ctx, "%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}
	tx_info->pkt_priv = pkt_priv;
	tx_info->data = data;
	tx_info->riid = riid;
	tx_info->size = size;
	tx_info->size_remaining = size;
	tx_info->iovec = iovec ? iovec : (void *)tx_info;
	tx_info->vprovider = vbuf_provider;
	tx_info->pprovider = pbuf_provider;
	GLINK_INFO_CH(ctx, "%s: data[%p], size[%u]. Thread: %u\n", __func__,
			tx_info->data ? tx_info->data : tx_info->iovec,
			tx_info->size, current->pid);

	/* schedule packet for transmit */
	if ((tx_flags & GLINK_TX_SINGLE_THREADED) &&
	    (ctx->transport_ptr->capabilities & GCAP_INTENTLESS))
		return xprt_single_threaded_tx(ctx->transport_ptr,
					       ctx, tx_info);
	else
		xprt_schedule_tx(ctx->transport_ptr, ctx, tx_info);

	return 0;
}

/**
 * glink_tx() - Transmit packet.
 *
 * @handle:	handle returned by glink_open()
 * @pkt_priv:	opaque data value that will be returned to client with
 *		notify_tx_done notification
 * @data:	pointer to the data
 * @size:	size of data
 * @tx_flags:	Flags to specify transmit specific options
 *
 * Return:	-EINVAL for invalid handle; -EBUSY if channel isn't ready for
 *		transmit operation (not fully opened); -EAGAIN if remote side
 *		has not provided a receive intent that is big enough.
 */
int glink_tx(void *handle, void *pkt_priv, void *data, size_t size,
							uint32_t tx_flags)
{
	return glink_tx_common(handle, pkt_priv, data, NULL, size,
			       tx_linear_vbuf_provider, NULL, tx_flags);
}
EXPORT_SYMBOL(glink_tx);

/**
 * glink_queue_rx_intent() - Register an intent to receive data.
 *
 * @handle:	handle returned by glink_open()
 * @pkt_priv:	opaque data type that is returned when a packet is received
 * size:	maximum size of data to receive
 *
 * Return: 0 for success; standard Linux error code for failure case
 */
int glink_queue_rx_intent(void *handle, const void *pkt_priv, size_t size)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;
	struct glink_core_rx_intent *intent_ptr;
	int ret;

	if (!ctx)
		return -EINVAL;

	if (!ch_is_fully_opened(ctx)) {
		/* Can only queue rx intents if channel is fully opened */
		GLINK_ERR_CH(ctx, "%s: Channel is not fully opened\n",
			__func__);
		return -EBUSY;
	}

	intent_ptr = ch_push_local_rx_intent(ctx, pkt_priv, size);
	if (!intent_ptr) {
		GLINK_ERR_CH(ctx,
			"%s: Intent pointer allocation failed size[%zu]\n",
			__func__, size);
		return -ENOMEM;
	}
	GLINK_DBG_CH(ctx, "%s: L[%u]:%zu\n", __func__, intent_ptr->id,
			intent_ptr->intent_size);

	/* notify remote side of rx intent */
	ret = ctx->transport_ptr->ops->tx_cmd_local_rx_intent(
		ctx->transport_ptr->ops, ctx->lcid, size, intent_ptr->id);
	if (ret)
		/* unable to transmit, dequeue intent */
		ch_remove_local_rx_intent(ctx, intent_ptr->id);

	return ret;
}
EXPORT_SYMBOL(glink_queue_rx_intent);

/**
 * glink_rx_done() - Return receive buffer to remote side.
 *
 * @handle:	handle returned by glink_open()
 * @ptr:	data pointer provided in the notify_rx() call
 * @reuse:	if true, receive intent is re-used
 *
 * Return: 0 for success; standard Linux error code for failure case
 */
int glink_rx_done(void *handle, const void *ptr)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;
	struct glink_core_rx_intent *liid_ptr;

	liid_ptr = ch_get_local_rx_intent_notified(ctx, ptr);

	if (IS_ERR_OR_NULL(liid_ptr)) {
		/* invalid pointer */
		GLINK_ERR_CH(ctx, "%s: Invalid pointer %p\n", __func__, ptr);
		return -EINVAL;
	}

	/* send rx done */
	ctx->transport_ptr->ops->tx_cmd_local_rx_done(ctx->transport_ptr->ops,
			ctx->lcid, liid_ptr->id);
	ctx->transport_ptr->ops->deallocate_rx_intent(liid_ptr);
	ch_remove_local_rx_intent_notified(ctx, liid_ptr);

	return 0;
}
EXPORT_SYMBOL(glink_rx_done);

/**
 * glink_txv() - Transmit a packet in vector form.
 *
 * @handle:	handle returned by glink_open()
 * @pkt_priv:	opaque data value that will be returned to client with
 *		notify_tx_done notification
 * @iovec:	pointer to the vector (must remain valid until notify_tx_done
 *		notification)
 * @size:	size of data/vector
 * @vbuf_provider: Client provided helper function to iterate the vector
 *		in physical address space
 * @pbuf_provider: Client provided helper function to iterate the vector
 *		in virtual address space
 * @tx_flags:	Flags to specify transmit specific options
 *
 * Return: -EINVAL for invalid handle; -EBUSY if channel isn't ready for
 *           transmit operation (not fully opened); -EAGAIN if remote side has
 *           not provided a receive intent that is big enough.
 */
int glink_txv(void *handle, void *pkt_priv,
	void *iovec, size_t size,
	void * (*vbuf_provider)(void *iovec, size_t offset, size_t *size),
	void * (*pbuf_provider)(void *iovec, size_t offset, size_t *size),
	uint32_t tx_flags)
{
	return glink_tx_common(handle, pkt_priv, NULL, iovec, size,
			vbuf_provider, pbuf_provider, tx_flags);
}
EXPORT_SYMBOL(glink_txv);

/**
 * glink_sigs_set() - Set the local signals for the GLINK channel
 *
 * @handle:	handle returned by glink_open()
 * @sigs:	modified signal value
 *
 * Return: 0 for success; standard Linux error code for failure case
 */
int glink_sigs_set(void *handle, uint32_t sigs)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;
	int ret;

	if (!ctx)
		return -EINVAL;

	if (!ch_is_fully_opened(ctx)) {
		GLINK_ERR_CH(ctx, "%s: Channel is not fully opened\n",
			__func__);
		return -EBUSY;
	}

	ctx->lsigs = sigs;

	ret = ctx->transport_ptr->ops->tx_cmd_set_sigs(ctx->transport_ptr->ops,
			ctx->lcid, ctx->lsigs);
	GLINK_INFO_CH(ctx, "%s: Sent SIGNAL SET command\n", __func__);

	return ret;
}
EXPORT_SYMBOL(glink_sigs_set);

/**
 * glink_sigs_local_get() - Get the local signals for the GLINK channel
 *
 * handle:	handle returned by glink_open()
 *
 * Return: Local signal value or standard Linux error code for failure case
 */
int glink_sigs_local_get(void *handle)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;

	if (!ctx)
		return -EINVAL;

	if (!ch_is_fully_opened(ctx)) {
		GLINK_ERR_CH(ctx, "%s: Channel is not fully opened\n",
			__func__);
		return -EBUSY;
	}

	return ctx->lsigs;
}
EXPORT_SYMBOL(glink_sigs_local_get);

/**
 * glink_sigs_remote_get() - Get the Remote signals for the GLINK channel
 *
 * handle:	handle returned by glink_open()
 *
 * Return: Remote signal value or standard Linux error code for failure case
 */
int glink_sigs_remote_get(void *handle)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;

	if (!ctx)
		return -EINVAL;

	if (!ch_is_fully_opened(ctx)) {
		GLINK_ERR_CH(ctx, "%s: Channel is not fully opened\n",
			__func__);
		return -EBUSY;
	}

	return ctx->rsigs;
}
EXPORT_SYMBOL(glink_sigs_remote_get);

/**
 * glink_register_link_state_cb() - Register for link state notification
 * @link_info:	Data structure containing the link identification and callback.
 * @priv:	Private information to be passed with the callback.
 *
 * This function is used to register a notifier to receive the updates about a
 * link's/transport's state. This notifier needs to be registered first before
 * an attempt to open a channel.
 *
 * Return: a reference to the notifier handle.
 */
void *glink_register_link_state_cb(struct glink_link_info *link_info,
				   void *priv)
{
	struct link_state_notifier_info *notif_info;

	if (!link_info || !link_info->glink_link_state_notif_cb)
		return ERR_PTR(-EINVAL);

	notif_info = kzalloc(sizeof(*notif_info), GFP_KERNEL);
	if (!notif_info) {
		GLINK_ERR("%s: Error allocating link state notifier info\n",
			  __func__);
		return ERR_PTR(-ENOMEM);
	}
	if (link_info->transport)
		strlcpy(notif_info->transport, link_info->transport,
			GLINK_NAME_SIZE);

	if (link_info->edge)
		strlcpy(notif_info->edge, link_info->edge, GLINK_NAME_SIZE);
	notif_info->priv = priv;
	notif_info->glink_link_state_notif_cb =
				link_info->glink_link_state_notif_cb;

	mutex_lock(&link_state_notifier_lock_lha1);
	list_add_tail(&notif_info->list, &link_state_notifier_list);
	mutex_unlock(&link_state_notifier_lock_lha1);

	notif_if_up_all_xprts(notif_info);
	return notif_info;
}
EXPORT_SYMBOL(glink_register_link_state_cb);

/**
 * glink_unregister_link_state_cb() - Unregister the link state notification
 * notif_handle:	Handle to be unregistered.
 *
 * This function is used to unregister a notifier to stop receiving the updates
 * about a link's/ transport's state.
 */
void glink_unregister_link_state_cb(void *notif_handle)
{
	struct link_state_notifier_info *notif_info, *tmp_notif_info;

	if (IS_ERR_OR_NULL(notif_handle))
		return;

	mutex_lock(&link_state_notifier_lock_lha1);
	list_for_each_entry_safe(notif_info, tmp_notif_info,
				 &link_state_notifier_list, list) {
		if (notif_info == notif_handle) {
			list_del(&notif_info->list);
			mutex_unlock(&link_state_notifier_lock_lha1);
			kfree(notif_info);
			return;
		}
	}
	mutex_unlock(&link_state_notifier_lock_lha1);
	return;
}
EXPORT_SYMBOL(glink_unregister_link_state_cb);

/**
 * glink_rpm_rx_poll() - Poll and receive any available packet
 * handle:	Channel handle in which this operation is performed.
 *
 * This function is used to poll and receive the packet while the receive
 * interrupt from RPM is disabled.
 *
 * Return: 0 on success, standard Linux error codes on failure.
 */
int glink_rpm_rx_poll(void *handle)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;

	if (!ctx)
		return -EINVAL;

	if (!ch_is_fully_opened(ctx))
		return -EBUSY;

	if (!ctx->transport_ptr ||
	    !(ctx->transport_ptr->capabilities & GCAP_INTENTLESS))
		return -EOPNOTSUPP;

	return ctx->transport_ptr->ops->poll(ctx->transport_ptr->ops,
					     ctx->lcid);
}
EXPORT_SYMBOL(glink_rpm_rx_poll);

/**
 * glink_rpm_mask_rx_interrupt() - Mask or unmask the RPM receive interrupt
 * handle:	Channel handle in which this operation is performed.
 * mask:	Flag to mask or unmask the interrupt.
 * pstruct:	Pointer to any platform specific data.
 *
 * This function is used to mask or unmask the receive interrupt from RPM.
 * "mask" set to true indicates masking the interrupt and when set to false
 * indicates unmasking the interrupt.
 *
 * Return: 0 on success, standard Linux error codes on failure.
 */
int glink_rpm_mask_rx_interrupt(void *handle, bool mask, void *pstruct)
{
	struct channel_ctx *ctx = (struct channel_ctx *)handle;

	if (!ctx)
		return -EINVAL;

	if (!ch_is_fully_opened(ctx))
		return -EBUSY;

	if (!ctx->transport_ptr ||
	    !(ctx->transport_ptr->capabilities & GCAP_INTENTLESS))
		return -EOPNOTSUPP;

	return ctx->transport_ptr->ops->mask_rx_irq(ctx->transport_ptr->ops,
						    ctx->lcid, mask, pstruct);

}
EXPORT_SYMBOL(glink_rpm_mask_rx_interrupt);

/**
 * glink_core_register_transport() - register a new transport
 * @if_ptr:	The interface to the transport.
 * @cfg:	Description and configuration of the transport.
 *
 * Return: 0 on success, EINVAL for invalid input.
 */
int glink_core_register_transport(struct glink_transport_if *if_ptr,
				  struct glink_core_transport_cfg *cfg)
{
	struct glink_core_xprt_ctx *xprt_ptr;
	size_t len;

	if (!if_ptr || !cfg || !cfg->name || !cfg->edge)
		return -EINVAL;

	len = strlen(cfg->name);
	if (len == 0 || len >= GLINK_NAME_SIZE)
		return -EINVAL;

	len = strlen(cfg->edge);
	if (len == 0 || len >= GLINK_NAME_SIZE)
		return -EINVAL;

	if (cfg->versions_entries < 1)
		return -EINVAL;

	xprt_ptr = kmalloc(sizeof(struct glink_core_xprt_ctx), GFP_KERNEL);
	if (xprt_ptr == NULL)
		return -ENOMEM;

	strlcpy(xprt_ptr->name, cfg->name, GLINK_NAME_SIZE);
	strlcpy(xprt_ptr->edge, cfg->edge, GLINK_NAME_SIZE);
	xprt_ptr->versions = cfg->versions;
	xprt_ptr->versions_entries = cfg->versions_entries;
	xprt_ptr->local_version_idx = cfg->versions_entries - 1;
	xprt_ptr->remote_version_idx = cfg->versions_entries - 1;
	if (!if_ptr->poll)
		if_ptr->poll = dummy_poll;
	if (!if_ptr->mask_rx_irq)
		if_ptr->mask_rx_irq = dummy_mask_rx_irq;
	xprt_ptr->capabilities = 0;
	xprt_ptr->ops = if_ptr;
	spin_lock_init(&xprt_ptr->xprt_ctx_lock_lhb1);
	xprt_ptr->next_lcid = 1; /* 0 reserved for default unconfigured */
	xprt_ptr->max_cid = cfg->max_cid;
	xprt_ptr->max_iid = cfg->max_iid;
	xprt_ptr->local_state = GLINK_XPRT_DOWN;
	xprt_ptr->remote_neg_completed = false;
	INIT_LIST_HEAD(&xprt_ptr->channels);
	INIT_LIST_HEAD(&xprt_ptr->tx_ready);
	mutex_init(&xprt_ptr->tx_ready_mutex_lhb2);
	INIT_WORK(&xprt_ptr->tx_work, tx_work_func);
	xprt_ptr->tx_wq = create_singlethread_workqueue("glink_tx");
	if (IS_ERR_OR_NULL(xprt_ptr->tx_wq)) {
		GLINK_ERR("%s: unable to allocate workqueue\n", __func__);
		kfree(xprt_ptr);
		return -ENOMEM;
	}

	if_ptr->glink_core_priv = xprt_ptr;
	if_ptr->glink_core_if_ptr = &core_impl;

	mutex_lock(&transport_list_lock_lha0);
	list_add_tail(&xprt_ptr->list_node, &transport_list);
	mutex_unlock(&transport_list_lock_lha0);
	glink_debugfs_add_xprt(xprt_ptr);

	return 0;
}
EXPORT_SYMBOL(glink_core_register_transport);

/**
 * glink_core_unregister_transport() - unregister a transport
 *
 * @if_ptr:	The interface to the transport.
 */
void glink_core_unregister_transport(struct glink_transport_if *if_ptr)
{
	/*
	 * FUTURE - transport unregister is not currently required and
	 * will be implemented as a follow-up change.
	 */
	GLINK_ERR("%s: not supported\n", __func__);
}
EXPORT_SYMBOL(glink_core_unregister_transport);

/**
 * glink_core_link_up() - transport link-up notification
 *
 * @if_ptr:	pointer to transport interface
 */
static void glink_core_link_up(struct glink_transport_if *if_ptr)
{
	struct glink_core_xprt_ctx *xprt_ptr = if_ptr->glink_core_priv;

	/* start local negotiation */
	xprt_ptr->local_state = GLINK_XPRT_NEGOTIATING;
	xprt_ptr->local_version_idx = xprt_ptr->versions_entries - 1;
	if_ptr->tx_cmd_version(if_ptr,
		    xprt_ptr->versions[xprt_ptr->local_version_idx].version,
		    xprt_ptr->versions[xprt_ptr->local_version_idx].features);

}

/**
 * glink_core_rx_cmd_version() - receive version/features from remote system
 *
 * @if_ptr:	pointer to transport interface
 * @r_version:	remote version
 * @r_features:	remote features
 *
 * This function is called in response to a remote-initiated version/feature
 * negotiation sequence.
 */
static void glink_core_rx_cmd_version(struct glink_transport_if *if_ptr,
	uint32_t r_version, uint32_t r_features)
{
	struct glink_core_xprt_ctx *xprt_ptr = if_ptr->glink_core_priv;
	const struct glink_core_version *versions = xprt_ptr->versions;
	bool neg_complete = false;
	uint32_t l_version, l_features;

	l_version = versions[xprt_ptr->remote_version_idx].version;
	l_features = versions[xprt_ptr->remote_version_idx].features;

	GLINK_INFO_XPRT(xprt_ptr,
		"%s: [local]%x:%08x [remote]%x:%08x\n", __func__,
		l_version, l_features, r_version, r_features);

	if (l_version > r_version) {
		/* Find matching version */
		while (true) {
			uint32_t rver_idx;

			if (xprt_ptr->remote_version_idx == 0) {
				/* version negotiation failed */
				GLINK_ERR_XPRT(xprt_ptr,
					"%s: Transport negotiation failed\n",
					__func__);
				l_version = 0;
				l_features = 0;
				break;
			}
			--xprt_ptr->remote_version_idx;
			rver_idx = xprt_ptr->remote_version_idx;

			if (versions[rver_idx].version <= r_version) {
				/* found a potential match */
				l_version = versions[rver_idx].version;
				l_features = versions[rver_idx].features;
				break;
			}
		}
	}

	if (l_version == r_version) {
		GLINK_INFO_XPRT(xprt_ptr,
			"%s: Remote and local version are matched %x:%08x\n",
			__func__, r_version, r_features);
		if (l_features != r_features) {
			uint32_t rver_idx = xprt_ptr->remote_version_idx;

			l_features = versions[rver_idx]
					.negotiate_features(if_ptr,
					&xprt_ptr->versions[rver_idx],
					r_features);
			GLINK_INFO_XPRT(xprt_ptr,
				"%s: negotiate features %x:%08x\n",
				__func__, l_version, l_features);
		}
		neg_complete = true;
	}
	if_ptr->tx_cmd_version_ack(if_ptr, l_version, l_features);

	if (neg_complete) {
		GLINK_INFO_XPRT(xprt_ptr,
			"%s: Remote negotiation complete %x:%08x\n", __func__,
			l_version, l_features);

		if (xprt_ptr->local_state == GLINK_XPRT_OPENED) {
			xprt_ptr->capabilities = if_ptr->set_version(if_ptr,
							l_version, l_features);
		}
		if_ptr->glink_core_priv->remote_neg_completed = true;
		if (xprt_is_fully_opened(xprt_ptr))
			check_link_notifier_and_notify(xprt_ptr,
						       GLINK_LINK_STATE_UP);
	}
}

/**
 * glink_core_rx_cmd_version_ack() - receive negotiation ack from remote system
 *
 * @if_ptr:	pointer to transport interface
 * @r_version:	remote version response
 * @r_features:	remote features response
 *
 * This function is called in response to a local-initiated version/feature
 * negotiation sequence and is the counter-offer from the remote side based
 * upon the initial version and feature set requested.
 */
static void glink_core_rx_cmd_version_ack(struct glink_transport_if *if_ptr,
	uint32_t r_version, uint32_t r_features)
{
	struct glink_core_xprt_ctx *xprt_ptr = if_ptr->glink_core_priv;
	const struct glink_core_version *versions = xprt_ptr->versions;
	uint32_t l_version, l_features;
	bool neg_complete = false;

	l_version = versions[xprt_ptr->local_version_idx].version;
	l_features = versions[xprt_ptr->local_version_idx].features;

	GLINK_INFO_XPRT(xprt_ptr,
		"%s: [local]%x:%08x [remote]%x:%08x\n", __func__,
		 l_version, l_features, r_version, r_features);

	if (l_version > r_version) {
		/* find matching version */
		while (true) {
			uint32_t lver_idx = xprt_ptr->local_version_idx;

			if (xprt_ptr->local_version_idx == 0) {
				/* version negotiation failed */
				xprt_ptr->local_state = GLINK_XPRT_FAILED;
				GLINK_ERR_XPRT(xprt_ptr,
					"%s: Transport negotiation failed\n",
					__func__);
				l_version = 0;
				l_features = 0;
				break;
			}
			--xprt_ptr->local_version_idx;
			lver_idx = xprt_ptr->local_version_idx;

			if (versions[lver_idx].version <= r_version) {
				/* found a potential match */
				l_version = versions[lver_idx].version;
				l_features = versions[lver_idx].features;
				break;
			}
		}
	} else if (l_version == r_version) {
		if (l_features != r_features) {
			/* version matches, negotiate features */
			uint32_t lver_idx = xprt_ptr->local_version_idx;

			l_features = versions[lver_idx].negotiate_features(
					if_ptr, &versions[lver_idx],
					r_features);
			GLINK_INFO_XPRT(xprt_ptr,
				"%s: negotiation features %x:%08x\n",
				__func__, l_version, l_features);
		} else {
			neg_complete = true;
		}
	} else {
		/*
		 * r_version > l_version
		 *
		 * Remote responded with a version greater than what we
		 * requested which is invalid and is treated as failure of the
		 * negotiation algorithm.
		 */
		GLINK_ERR_XPRT(xprt_ptr,
			"%s: [local]%x:%08x [remote]%x:%08x neg failure\n",
			__func__, l_version, l_features, r_version,
			r_features);
		xprt_ptr->local_state = GLINK_XPRT_FAILED;
		l_version = 0;
		l_features = 0;
	}

	if (neg_complete) {
		/* negotiation complete */
		GLINK_INFO_XPRT(xprt_ptr,
			"%s: Local negotiation complete %x:%08x\n",
			__func__, l_version, l_features);

		if (xprt_ptr->remote_neg_completed) {
			xprt_ptr->capabilities = if_ptr->set_version(if_ptr,
							l_version, l_features);
		}

		xprt_ptr->local_state = GLINK_XPRT_OPENED;
		if (xprt_is_fully_opened(xprt_ptr))
			check_link_notifier_and_notify(xprt_ptr,
						       GLINK_LINK_STATE_UP);
	} else {
		if_ptr->tx_cmd_version(if_ptr, l_version, l_features);
	}
}

/**
 * glink_core_rx_cmd_ch_remote_open() - Remote-initiated open command
 *
 * @if_ptr:	Pointer to transport instance
 * @rcid:	Remote Channel ID
 * @name:	Channel name
 */
static void glink_core_rx_cmd_ch_remote_open(struct glink_transport_if *if_ptr,
	uint32_t rcid, const char *name)
{
	struct channel_ctx *ctx;

	ctx = ch_name_to_ch_ctx_create(if_ptr->glink_core_priv, name);
	if (ctx == NULL) {
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
		       "%s: invalid rcid %u received, name '%s'\n",
		       __func__, rcid, name);
		return;
	}

	/* port already exists */
	if (ctx->remote_opened) {
		GLINK_ERR_CH(ctx,
		       "%s: Duplicate remote open for rcid %u, name '%s'\n",
		       __func__, rcid, name);
		return;
	}

	ctx->remote_opened = true;
	ch_add_rcid(if_ptr->glink_core_priv, ctx, rcid);
	ctx->transport_ptr = if_ptr->glink_core_priv;

	if (ch_is_fully_opened(ctx))
		ctx->notify_state(ctx, ctx->user_priv, GLINK_CONNECTED);

	if_ptr->tx_cmd_ch_remote_open_ack(if_ptr, rcid);
}

/**
 * glink_core_rx_cmd_ch_open_ack() - Receive ack to previously sent open request
 *
 * if_ptr:	Pointer to transport instance
 * lcid:	Local Channel ID
 */
static void glink_core_rx_cmd_ch_open_ack(struct glink_transport_if *if_ptr,
	uint32_t lcid)
{
	struct channel_ctx *ctx;

	ctx = xprt_lcid_to_ch_ctx(if_ptr->glink_core_priv, lcid);
	if (!ctx) {
		/* unknown LCID received - this shouldn't happen */
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: invalid lcid %u received\n", __func__,
				(unsigned)lcid);
		return;
	}

	if (ctx->local_open_state != GLINK_CHANNEL_OPENING) {
		GLINK_ERR_CH(ctx,
			"%s: unexpected open ack receive for lcid. Current state: %u. Thread: %u\n",
				__func__, ctx->local_open_state, current->pid);
		return;
	}

	ctx->local_open_state = GLINK_CHANNEL_OPENED;
	GLINK_INFO_PERF_CH(ctx,
		"%s: local:GLINK_CHANNEL_OPENING_WAIT->GLINK_CHANNEL_OPENED\n",
		__func__);
	if (ch_is_fully_opened(ctx)) {
		ctx->notify_state(ctx, ctx->user_priv, GLINK_CONNECTED);
		GLINK_INFO_PERF_CH(ctx, "%s: notify state: GLINK_CONNECTED\n",
				__func__);
	}
}

/**
 * glink_core_rx_cmd_ch_remote_close() - Receive remote close command
 *
 * if_ptr:	Pointer to transport instance
 * rcid:	Remote Channel ID
 */
static void glink_core_rx_cmd_ch_remote_close(
		struct glink_transport_if *if_ptr, uint32_t rcid)
{
	struct channel_ctx *ctx;

	ctx = xprt_rcid_to_ch_ctx(if_ptr->glink_core_priv, rcid);
	if (!ctx) {
		/* unknown LCID received - this shouldn't happen */
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: invalid rcid %u received\n", __func__,
				(unsigned)rcid);
		return;
	}

	if (!ctx->remote_opened) {
		GLINK_ERR_CH(ctx,
			"%s: unexpected remote close receive for rcid %u\n",
			__func__, (unsigned)rcid);
		return;
	}

	glink_core_remote_close_common(ctx);

	if_ptr->tx_cmd_ch_remote_close_ack(if_ptr, rcid);
}

/**
 * glink_core_rx_cmd_ch_close_ack() - Receive locally-request close ack
 *
 * if_ptr:	Pointer to transport instance
 * rcid:	Remote Channel ID
 */
static void glink_core_rx_cmd_ch_close_ack(struct glink_transport_if *if_ptr,
	uint32_t lcid)
{
	struct channel_ctx *ctx;

	ctx = xprt_lcid_to_ch_ctx(if_ptr->glink_core_priv, lcid);
	if (!ctx) {
		/* unknown LCID received - this shouldn't happen */
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: invalid lcid %u received\n", __func__,
				(unsigned)lcid);
		return;
	}

	if (ctx->local_open_state != GLINK_CHANNEL_CLOSING) {
		GLINK_ERR_CH(ctx,
			"%s: unexpected close ack receive for lcid %u\n",
			__func__, (unsigned)lcid);
		return;
	}

	ctx->local_open_state = GLINK_CHANNEL_CLOSED;
	GLINK_INFO_PERF_CH(ctx,
		"%s: local:GLINK_CHANNEL_CLOSING->GLINK_CHANNEL_CLOSED\n",
		__func__);

	if (ctx->notify_state) {
		ctx->notify_state(ctx, ctx->user_priv,
				GLINK_LOCAL_DISCONNECTED);
		ch_purge_intent_lists(ctx);
		GLINK_INFO_PERF_CH(ctx,
				"%s: notify state: GLINK_LOCAL_DISCONNECTED\n",
				__func__);
	}
}

/**
 * glink_core_remote_rx_intent_put() - Receive remove intent
 *
 * @if_ptr:	Pointer to transport instance
 * @rcid:	Remote Channel ID
 * @riid:	Remote Intent ID
 * @size:	Size of the remote intent ID
 */
static void glink_core_remote_rx_intent_put(struct glink_transport_if *if_ptr,
		uint32_t rcid, uint32_t riid, size_t size)
{
	struct channel_ctx *ctx;

	ctx = xprt_rcid_to_ch_ctx(if_ptr->glink_core_priv, rcid);
	if (!ctx) {
		/* unknown rcid received - this shouldn't happen */
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: invalid rcid received %u\n", __func__,
				(unsigned)rcid);
		return;
	}

	ch_push_remote_rx_intent(ctx, size, riid);
	complete_all(&ctx->int_req_complete);
}

/**
 * glink_core_rx_cmd_remote_rx_intent_req() - Receive a request for rx_intent
 *                                            from remote side
 * if_ptr:	Pointer to the transport interface
 * rcid:	Remote channel ID
 * size:	size of the intent
 *
 * The function searches for the local channel to which the request for
 * rx_intent has arrived and informs this request to the local channel through
 * notify_rx_intent_req callback registered by the local channel.
 */
static void glink_core_rx_cmd_remote_rx_intent_req(
	struct glink_transport_if *if_ptr, uint32_t rcid, size_t size)
{
	struct channel_ctx *ctx;
	bool cb_ret;

	ctx = xprt_rcid_to_ch_ctx(if_ptr->glink_core_priv, rcid);
	if (!ctx) {
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: invalid rcid received %u\n", __func__,
				(unsigned)rcid);
		return;
	}
	if (!ctx->notify_rx_intent_req) {
		GLINK_ERR_CH(ctx,
			"%s: Notify function not defined for local channel",
			__func__);
		return;
	}

	cb_ret = ctx->notify_rx_intent_req(ctx, ctx->user_priv, size);
	if_ptr->tx_cmd_remote_rx_intent_req_ack(if_ptr, ctx->lcid, cb_ret);
}

/**
 * glink_core_rx_cmd_remote_rx_intent_req_ack()- Receive ack from remote side
 *						for a local rx_intent request
 * if_ptr:	Pointer to the transport interface
 * rcid:	Remote channel ID
 * size:	size of the intent
 *
 * This function receives the ack for rx_intent request from local channel.
 */
static void glink_core_rx_cmd_rx_intent_req_ack(struct glink_transport_if
					*if_ptr, uint32_t rcid, bool granted)
{
	struct channel_ctx *ctx;

	ctx = xprt_rcid_to_ch_ctx(if_ptr->glink_core_priv, rcid);
	if (!ctx) {
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: Invalid rcid received %u\n", __func__,
				(unsigned)rcid);
		return;
	}
	ctx->int_req_ack = granted;
	complete_all(&ctx->int_req_ack_complete);
}

/**
 * glink_core_rx_get_pkt_ctx() - lookup RX intent structure
 *
 * if_ptr:	Pointer to the transport interface
 * rcid:	Remote channel ID
 * liid:	Local RX Intent ID
 *
 * Note that this function is designed to always be followed by a call to
 * glink_core_rx_put_pkt_ctx() to complete an RX operation by the transport.
 *
 * Return: Pointer to RX intent structure (or NULL if none found)
 */
static struct glink_core_rx_intent *glink_core_rx_get_pkt_ctx(
		struct glink_transport_if *if_ptr, uint32_t rcid, uint32_t liid)
{
	struct channel_ctx *ctx;
	struct glink_core_rx_intent *intent_ptr;

	ctx = xprt_rcid_to_ch_ctx(if_ptr->glink_core_priv, rcid);
	if (!ctx) {
		/* unknown LCID received - this shouldn't happen */
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: invalid rcid received %u\n", __func__,
				(unsigned)rcid);
		return NULL;
	}

	/* match pending intent */
	intent_ptr = ch_get_local_rx_intent(ctx, liid);
	if (intent_ptr == NULL) {
		GLINK_ERR_CH(ctx,
			"%s: L[%u]: No matching rx intent\n",
			__func__, liid);
		return NULL;
	}

	return intent_ptr;
}

/**
 * glink_core_rx_put_pkt_ctx() - lookup RX intent structure
 *
 * if_ptr:	Pointer to the transport interface
 * rcid:	Remote channel ID
 * intent_ptr:	Pointer to the RX intent
 * complete:	Packet has been completely received
 *
 * Note that this function should always be preceded by a call to
 * glink_core_rx_get_pkt_ctx().
 */
void glink_core_rx_put_pkt_ctx(struct glink_transport_if *if_ptr,
	uint32_t rcid, struct glink_core_rx_intent *intent_ptr, bool complete)
{
	struct channel_ctx *ctx;

	if (!complete) {
		GLINK_DBG_XPRT(if_ptr->glink_core_priv,
			"%s: rcid[%u] liid[%u] pkt_size[%zu] write_offset[%zu] Fragment received\n",
				__func__, rcid, intent_ptr->id,
				intent_ptr->pkt_size,
				intent_ptr->write_offset);
		return;
	}
	GLINK_DBG_XPRT(if_ptr->glink_core_priv,
		"%s: rcid[%u] liid[%u] pkt_size[%zu] write_offset[%zu] Complete packet received\n",
			__func__, rcid, intent_ptr->id,
			intent_ptr->pkt_size,
			intent_ptr->write_offset);

	/* packet complete */
	ctx = xprt_rcid_to_ch_ctx(if_ptr->glink_core_priv, rcid);
	if (!ctx) {
		/* unknown LCID received - this shouldn't happen */
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
			       "%s: invalid rcid received %u\n", __func__,
			       (unsigned)rcid);
		return;
	}

	if (!intent_ptr->data && !ctx->notify_rxv) {
		/* Received a vector, but client can't handle a vector */
		intent_ptr->bounce_buf = linearize_vector(intent_ptr->iovec,
						intent_ptr->pkt_size,
						intent_ptr->vprovider,
						intent_ptr->pprovider);
		if (IS_ERR_OR_NULL(intent_ptr->bounce_buf)) {
			GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: Error %ld linearizing vector\n", __func__,
				PTR_ERR(intent_ptr->bounce_buf));
			BUG();
			return;
		}
	}

	ch_set_local_rx_intent_notified(ctx, intent_ptr);
	if (ctx->notify_rx && (intent_ptr->data || intent_ptr->bounce_buf)) {
		ctx->notify_rx(ctx, ctx->user_priv, intent_ptr->pkt_priv,
			       intent_ptr->data ?
				intent_ptr->data : intent_ptr->bounce_buf,
			       intent_ptr->pkt_size);
	} else if (ctx->notify_rxv) {
		ctx->notify_rxv(ctx, ctx->user_priv, intent_ptr->pkt_priv,
				intent_ptr->iovec, intent_ptr->pkt_size,
				intent_ptr->vprovider, intent_ptr->pprovider);
	} else {
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: Unable to process rx data\n", __func__);
		BUG();
	}
}

/**
 * glink_core_rx_cmd_tx_done() - Receive Transmit Done Command
 * @xprt_ptr:	Transport to send packet on.
 * @rcid:	Remote channel ID
 * @riid:	Remote intent ID
 */
void glink_core_rx_cmd_tx_done(struct glink_transport_if *if_ptr, uint32_t
		rcid, uint32_t riid)
{
	struct channel_ctx *ctx;
	struct glink_core_tx_pkt *tx_pkt;

	ctx = xprt_rcid_to_ch_ctx(if_ptr->glink_core_priv, rcid);
	if (!ctx) {
		/* unknown RCID received - this shouldn't happen */
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: invalid rcid %u received\n", __func__,
				rcid);
		return;
	}

	mutex_lock(&ctx->tx_lists_mutex_lhc3);
	tx_pkt = ch_get_tx_pending_remote_done(ctx, riid);
	if (IS_ERR_OR_NULL(tx_pkt)) {
		/*
		 * FUTURE - in the case of a zero-copy transport, this is a
		 * fatal protocol failure since memory corruption could occur
		 * in this case.  Prevent this by adding code in glink_close()
		 * to recall any buffers in flight / wait for them to be
		 * returned.
		 */
		GLINK_ERR_CH(ctx, "%s: R[%u]: No matching tx\n",
				__func__,
				(unsigned)riid);
		return;
	}

	/* notify client */
	ctx->notify_tx_done(ctx, ctx->user_priv, tx_pkt->pkt_priv,
			    tx_pkt->data ? tx_pkt->data : tx_pkt->iovec);
	ch_remove_tx_pending_remote_done(ctx, tx_pkt);
	mutex_unlock(&ctx->tx_lists_mutex_lhc3);
}

/**
 * xprt_schedule_tx() - Schedules packet for transmit.
 * @xprt_ptr:	Transport to send packet on.
 * @ch_ptr:	Channel to send packet on.
 * @tx_info:	Packet to transmit.
 */
static void xprt_schedule_tx(struct glink_core_xprt_ctx *xprt_ptr,
			     struct channel_ctx *ch_ptr,
			     struct glink_core_tx_pkt *tx_info)
{
	mutex_lock(&xprt_ptr->tx_ready_mutex_lhb2);
	if (list_empty(&ch_ptr->tx_ready_list_node))
		list_add_tail(&ch_ptr->tx_ready_list_node, &xprt_ptr->tx_ready);

	mutex_lock(&ch_ptr->tx_lists_mutex_lhc3);
	list_add_tail(&tx_info->list_node, &ch_ptr->tx_active);
	mutex_unlock(&ch_ptr->tx_lists_mutex_lhc3);
	mutex_unlock(&xprt_ptr->tx_ready_mutex_lhb2);

	queue_work(xprt_ptr->tx_wq, &xprt_ptr->tx_work);
}

/**
 * xprt_single_threaded_tx() - Transmit in the context of sender.
 * @xprt_ptr:	Transport to send packet on.
 * @ch_ptr:	Channel to send packet on.
 * @tx_info:	Packet to transmit.
 */
static int xprt_single_threaded_tx(struct glink_core_xprt_ctx *xprt_ptr,
			     struct channel_ctx *ch_ptr,
			     struct glink_core_tx_pkt *tx_info)
{
	int ret;

	mutex_lock(&ch_ptr->tx_lists_mutex_lhc3);
	do {
		ret = xprt_ptr->ops->tx(ch_ptr->transport_ptr->ops,
					ch_ptr->lcid, tx_info);
	} while (ret == -EAGAIN);
	if (ret < 0 || tx_info->size_remaining) {
		GLINK_ERR_CH(ch_ptr, "%s: Error %d writing data\n",
			     __func__, ret);
		kfree(tx_info);
	} else {
		list_add_tail(&tx_info->list_node,
			      &ch_ptr->tx_pending_remote_done);
		ret = 0;
	}
	mutex_unlock(&ch_ptr->tx_lists_mutex_lhc3);
	return ret;
}


/**
 * tx_work_func() - Transmit worker
 * @work:	Linux work structure
 */
static void tx_work_func(struct work_struct *work)
{
	struct glink_core_xprt_ctx *xprt_ptr =
			container_of(work, struct glink_core_xprt_ctx, tx_work);
	struct channel_ctx *ch_ptr;
	struct glink_core_tx_pkt *tx_info;
	int ret;

	mutex_lock(&xprt_ptr->tx_ready_mutex_lhb2);
	while (!list_empty(&xprt_ptr->tx_ready)) {

		ch_ptr = list_first_entry(&xprt_ptr->tx_ready,
				struct channel_ctx,
				tx_ready_list_node);
		mutex_unlock(&xprt_ptr->tx_ready_mutex_lhb2);

		mutex_lock(&ch_ptr->tx_lists_mutex_lhc3);
		tx_info = list_first_entry(&ch_ptr->tx_active,
					struct glink_core_tx_pkt,
					list_node);

		ret = xprt_ptr->ops->tx(xprt_ptr->ops, ch_ptr->lcid, tx_info);
		if (ret == -EAGAIN) {
			/*
			 * transport unable to send at the moment and will call
			 * tx_resume() when it can send again.
			 */
			break;
		} else if (ret < 0) {
			/*
			 * General failure code that indicates that the
			 * transport is unable to recover.  In this case, the
			 * communication failure will be detected at a higher
			 * level and a subsystem restart of the affected system
			 * will be triggered.
			 */
			GLINK_ERR_XPRT(xprt_ptr,
					"%s: unrecoverable xprt failure %d\n",
					__func__, ret);
		}

		if (!tx_info->size_remaining)
			list_move(&tx_info->list_node,
				&ch_ptr->tx_pending_remote_done);

		if (list_empty(&ch_ptr->tx_active)) {
			mutex_unlock(&ch_ptr->tx_lists_mutex_lhc3);
			mutex_lock(&xprt_ptr->tx_ready_mutex_lhb2);
			mutex_lock(&ch_ptr->tx_lists_mutex_lhc3);
			if (list_empty(&ch_ptr->tx_active))
				list_del_init(&ch_ptr->tx_ready_list_node);
			mutex_unlock(&xprt_ptr->tx_ready_mutex_lhb2);
		}
		mutex_unlock(&ch_ptr->tx_lists_mutex_lhc3);

		mutex_lock(&xprt_ptr->tx_ready_mutex_lhb2);
		list_rotate_left(&xprt_ptr->tx_ready);
	}
	mutex_unlock(&xprt_ptr->tx_ready_mutex_lhb2);
}

static void glink_core_tx_resume(struct glink_transport_if *if_ptr)
{
	queue_work(if_ptr->glink_core_priv->tx_wq,
					&if_ptr->glink_core_priv->tx_work);
}

/**
 * glink_core_rx_cmd_remote_sigs() - Receive remote channel signal command
 *
 * if_ptr:	Pointer to transport instance
 * rcid:	Remote Channel ID
 */
static void glink_core_rx_cmd_remote_sigs(struct glink_transport_if *if_ptr,
					uint32_t rcid, uint32_t sigs)
{
	struct channel_ctx *ctx;
	uint32_t old_sigs;

	ctx = xprt_rcid_to_ch_ctx(if_ptr->glink_core_priv, rcid);
	if (!ctx) {
		/* unknown LCID received - this shouldn't happen */
		GLINK_ERR_XPRT(if_ptr->glink_core_priv,
				"%s: invalid rcid %u received\n", __func__,
				(unsigned)rcid);
		return;
	}

	if (!ch_is_fully_opened(ctx)) {
		GLINK_ERR_CH(ctx, "%s: Channel is not fully opened\n",
			__func__);
		return;
	}

	old_sigs = ctx->rsigs;
	ctx->rsigs = sigs;
	if (ctx->notify_rx_sigs) {
		ctx->notify_rx_sigs(ctx, ctx->user_priv, old_sigs, ctx->rsigs);
		GLINK_INFO_CH(ctx, "%s: notify rx sigs old:0x%x new:0x%x\n",
				__func__, old_sigs, ctx->rsigs);
	}
}

static struct glink_core_if core_impl = {
	.link_up = glink_core_link_up,
	.rx_cmd_version = glink_core_rx_cmd_version,
	.rx_cmd_version_ack = glink_core_rx_cmd_version_ack,
	.rx_cmd_ch_remote_open = glink_core_rx_cmd_ch_remote_open,
	.rx_cmd_ch_open_ack = glink_core_rx_cmd_ch_open_ack,
	.rx_cmd_ch_remote_close = glink_core_rx_cmd_ch_remote_close,
	.rx_cmd_ch_close_ack = glink_core_rx_cmd_ch_close_ack,
	.rx_get_pkt_ctx = glink_core_rx_get_pkt_ctx,
	.rx_put_pkt_ctx = glink_core_rx_put_pkt_ctx,
	.rx_cmd_remote_rx_intent_put = glink_core_remote_rx_intent_put,
	.rx_cmd_remote_rx_intent_req = glink_core_rx_cmd_remote_rx_intent_req,
	.rx_cmd_rx_intent_req_ack = glink_core_rx_cmd_rx_intent_req_ack,
	.rx_cmd_tx_done = glink_core_rx_cmd_tx_done,
	.tx_resume = glink_core_tx_resume,
	.rx_cmd_remote_sigs = glink_core_rx_cmd_remote_sigs,
};

/**
 * glink_xprt_ctx_iterator_init() - Initializes the transport context list iterator
 * @xprt_i:	pointer to the transport context iterator.
 *
 * This function acquires the transport context lock which must then be
 * released by glink_xprt_ctx_iterator_end()
 */
void glink_xprt_ctx_iterator_init(struct xprt_ctx_iterator *xprt_i)
{
	if (xprt_i == NULL)
		return;

	mutex_lock(&transport_list_lock_lha0);
	xprt_i->xprt_list = &transport_list;
	xprt_i->i_curr = list_entry(&transport_list,
			struct glink_core_xprt_ctx, list_node);
}
EXPORT_SYMBOL(glink_xprt_ctx_iterator_init);

/**
 * glink_xprt_ctx_iterator_end() - Ends the transport context list iteration
 * @xprt_i:	pointer to the transport context iterator.
 */
void glink_xprt_ctx_iterator_end(struct xprt_ctx_iterator *xprt_i)
{
	if (xprt_i == NULL)
		return;

	xprt_i->xprt_list = NULL;
	xprt_i->i_curr = NULL;
	mutex_unlock(&transport_list_lock_lha0);
}
EXPORT_SYMBOL(glink_xprt_ctx_iterator_end);

/**
 * glink_xprt_ctx_iterator_next() - iterates element by element in transport context list
 * @xprt_i:	pointer to the transport context iterator.
 *
 * Return: pointer to the transport context structure
 */
struct glink_core_xprt_ctx *glink_xprt_ctx_iterator_next(
			struct xprt_ctx_iterator *xprt_i)
{
	struct glink_core_xprt_ctx *xprt_ctx = NULL;

	if (xprt_i == NULL)
		return xprt_ctx;

	if (list_empty(xprt_i->xprt_list))
		return xprt_ctx;

	list_for_each_entry_continue(xprt_i->i_curr,
			xprt_i->xprt_list, list_node) {
		xprt_ctx = xprt_i->i_curr;
		break;
	}
	return xprt_ctx;
}
EXPORT_SYMBOL(glink_xprt_ctx_iterator_next);

/**
 * glink_get_xprt_name() - get the transport name
 * @xprt_ctx:	pointer to the transport context.
 *
 * Return: name of the transport
 */
char *glink_get_xprt_name(struct glink_core_xprt_ctx *xprt_ctx)
{
	if (xprt_ctx == NULL)
		return NULL;

	return xprt_ctx->name;
}
EXPORT_SYMBOL(glink_get_xprt_name);

/**
 * glink_get_xprt_name() - get the name of the remote processor/edge
 *				of the transport
 * @xprt_ctx:	pointer to the transport context.
 *
 * Return: Name of the remote processor/edge
 */
char *glink_get_xprt_edge_name(struct glink_core_xprt_ctx *xprt_ctx)
{
	if (xprt_ctx == NULL)
		return NULL;
	return xprt_ctx->edge;
}
EXPORT_SYMBOL(glink_get_xprt_edge_name);

/**
 * glink_get_xprt_state() - get the state of the transport
 * @xprt_ctx:	pointer to the transport context.
 *
 * Return: Name of the transport state, NULL in case of invalid input
 */
const char *glink_get_xprt_state(struct glink_core_xprt_ctx *xprt_ctx)
{
	if (xprt_ctx == NULL)
		return NULL;

	return glink_get_xprt_state_string(xprt_ctx->local_state);
}
EXPORT_SYMBOL(glink_get_xprt_state);

/**
 * glink_get_xprt_version_features() - get the version and feature set
 *					of local transport in glink
 * @xprt_ctx:	pointer to the transport context.
 *
 * Return: pointer to the glink_core_version
 */
const struct glink_core_version *glink_get_xprt_version_features(
		struct glink_core_xprt_ctx *xprt_ctx)
{
	const struct glink_core_version *ver = NULL;
	if (xprt_ctx == NULL)
		return ver;

	ver = &xprt_ctx->versions[xprt_ctx->local_version_idx];
	return ver;
}
EXPORT_SYMBOL(glink_get_xprt_version_features);

/**
 * glink_ch_ctx_iterator_init() - Initializes the channel context list iterator
 * @ch_iter:	pointer to the channel context iterator.
 * xprt:	pointer to the transport context that holds the channel list
 *
 * This function acquires the channel context lock which must then be
 * released by glink_ch_ctx_iterator_end()
 */
void glink_ch_ctx_iterator_init(struct ch_ctx_iterator *ch_iter,
		struct glink_core_xprt_ctx *xprt)
{
	unsigned long flags;

	if (ch_iter == NULL || xprt == NULL)
		return;

	spin_lock_irqsave(&xprt->xprt_ctx_lock_lhb1, flags);
	ch_iter->ch_list = &(xprt->channels);
	ch_iter->i_curr = list_entry(&(xprt->channels),
				struct channel_ctx, port_list_node);
	ch_iter->ch_list_flags = flags;
}
EXPORT_SYMBOL(glink_ch_ctx_iterator_init);

/**
 * glink_ch_ctx_iterator_end() - Ends the channel context list iteration
 * @ch_iter:	pointer to the channel context iterator.
 */
void glink_ch_ctx_iterator_end(struct ch_ctx_iterator *ch_iter,
				struct glink_core_xprt_ctx *xprt)
{
	if (ch_iter == NULL || xprt == NULL)
		return;

	spin_unlock_irqrestore(&xprt->xprt_ctx_lock_lhb1,
			ch_iter->ch_list_flags);
	ch_iter->ch_list = NULL;
	ch_iter->i_curr = NULL;
}
EXPORT_SYMBOL(glink_ch_ctx_iterator_end);

/**
 * glink_ch_ctx_iterator_next() - iterates element by element in channel context list
 * @c_i:	pointer to the channel context iterator.
 *
 * Return: pointer to the channel context structure
 */
struct channel_ctx *glink_ch_ctx_iterator_next(struct ch_ctx_iterator *c_i)
{
	struct channel_ctx *ch_ctx = NULL;

	if (c_i == NULL)
		return ch_ctx;

	if (list_empty(c_i->ch_list))
		return ch_ctx;

	list_for_each_entry_continue(c_i->i_curr,
			c_i->ch_list, port_list_node) {
		ch_ctx = c_i->i_curr;
		break;
	}
	return ch_ctx;
}
EXPORT_SYMBOL(glink_ch_ctx_iterator_next);

/**
 * glink_get_ch_name() - get the channel name
 * @ch_ctx:	pointer to the channel context.
 *
 * Return: name of the channel, NULL in case of invalid input
 */
char *glink_get_ch_name(struct channel_ctx *ch_ctx)
{
	if (ch_ctx == NULL)
		return NULL;

	return ch_ctx->name;
}
EXPORT_SYMBOL(glink_get_ch_name);

/**
 * glink_get_ch_edge_name() - get the edge on whcih channel is created
 * @ch_ctx:	pointer to the channel context.
 *
 * Return: name of the edge, NULL in case of invalid input
 */
char *glink_get_ch_edge_name(struct channel_ctx *ch_ctx)
{
	if (ch_ctx == NULL)
		return NULL;

	return ch_ctx->transport_ptr->edge;
}
EXPORT_SYMBOL(glink_get_ch_edge_name);

/**
 * glink_get_ch_lcid() - get the local channel ID
 * @c_i:	pointer to the channel context.
 *
 * Return: local channel id, -EINVAL in case of invalid input
 */
int glink_get_ch_lcid(struct channel_ctx *ch_ctx)
{
	if (ch_ctx == NULL)
		return -EINVAL;

	return ch_ctx->lcid;
}
EXPORT_SYMBOL(glink_get_ch_lcid);

/**
 * glink_get_ch_rcid() - get the remote channel ID
 * @ch_ctx:	pointer to the channel context.
 *
 * Return: remote channel id, -EINVAL in case of invalid input
 */
int glink_get_ch_rcid(struct channel_ctx *ch_ctx)
{
	if (ch_ctx == NULL)
		return -EINVAL;

	return ch_ctx->rcid;
}
EXPORT_SYMBOL(glink_get_ch_rcid);

/**
 * glink_get_ch_state() - get the channel state
 * @ch_ctx:	pointer to the channel context.
 *
 * Return: Name of the channel state, NUll in case of invalid input
 */
const char *glink_get_ch_state(struct channel_ctx *ch_ctx)
{
	if (ch_ctx == NULL)
		return NULL;

	return glink_get_ch_state_string(ch_ctx->local_open_state);
}
EXPORT_SYMBOL(glink_get_ch_state);

/**
 * glink_get_ch_xprt_name() - get the name of the transport to which
 *				the channel belongs
 * @ch_ctx:	pointer to the channel context.
 *
 * Return: name of the export, NULL in case of invalid input
 */
char *glink_get_ch_xprt_name(struct channel_ctx *ch_ctx)
{
	if (ch_ctx == NULL)
		return NULL;

	return ch_ctx->transport_ptr->name;
}
EXPORT_SYMBOL(glink_get_ch_xprt_name);

/**
 * glink_get_tx_pkt_count() - get the total number of packets sent
 *				through this channel
 * @ch_ctx:	pointer to the channel context.
 *
 * Return: number of packets transmitted, -EINVAL in case of invalid input
 */
int glink_get_ch_tx_pkt_count(struct channel_ctx *ch_ctx)
{
	if (ch_ctx == NULL)
		return -EINVAL;

	/* FUTURE: packet stats not yet implemented */

	return -ENOSYS;
}
EXPORT_SYMBOL(glink_get_ch_tx_pkt_count);

/**
 * glink_get_ch_rx_pkt_count() - get the total number of packets
 *				recieved at this channel
 * @ch_ctx:	pointer to the channel context.
 *
 * Return: number of packets recieved, -EINVAL in case of invalid input
 */
int glink_get_ch_rx_pkt_count(struct channel_ctx *ch_ctx)
{
	if (ch_ctx == NULL)
		return -EINVAL;

	/* FUTURE: packet stats not yet implemented */

	return -ENOSYS;
}
EXPORT_SYMBOL(glink_get_ch_rx_pkt_count);

/**
 * glink_get_ch_lintents_queued() - get the total number of intents queued
 *				at local side
 * @ch_ctx:	pointer to the channel context.
 *
 * Return: number of intents queued, -EINVAL in case of invalid input
 */
int glink_get_ch_lintents_queued(struct channel_ctx *ch_ctx)
{
	struct glink_core_rx_intent *intent;
	int ilrx_count = 0;

	if (ch_ctx == NULL)
		return -EINVAL;

	list_for_each_entry(intent, &ch_ctx->local_rx_intent_list, list)
		ilrx_count++;

	return ilrx_count;
}
EXPORT_SYMBOL(glink_get_ch_lintents_queued);

/**
 * glink_get_ch_rintents_queued() - get the total number of intents queued
 *				from remote side
 * @ch_ctx:	pointer to the channel context.
 *
 * Return: number of intents queued, -EINVAL in case of invalid input
 */
int glink_get_ch_rintents_queued(struct channel_ctx *ch_ctx)
{
	struct glink_core_rx_intent *intent;
	int irrx_count = 0;

	if (ch_ctx == NULL)
		return -EINVAL;

	list_for_each_entry(intent, &ch_ctx->rmt_rx_intent_list, list)
		irrx_count++;

	return irrx_count;
}
EXPORT_SYMBOL(glink_get_ch_rintents_queued);

/**
 * glink_get_ch_intent_info() - get the intent details of a channel
 * @ch_ctx:	pointer to the channel context.
 * ch_ctx_i:	pointer to a structure that will contain intent details
 *
 * This function is used to get all the channel intent details including locks.
 */
void glink_get_ch_intent_info(struct channel_ctx *ch_ctx,
			struct glink_ch_intent_info *ch_ctx_i)
{
	if (ch_ctx == NULL || ch_ctx_i == NULL)
		return;

	ch_ctx_i->li_lst_lock = &ch_ctx->local_rx_intent_lst_lock_lhc1;
	ch_ctx_i->li_avail_list = &ch_ctx->local_rx_intent_list;
	ch_ctx_i->li_used_list = &ch_ctx->local_rx_intent_ntfy_list;
	ch_ctx_i->ri_lst_lock = &ch_ctx->rmt_rx_intent_lst_lock_lhc2;
	ch_ctx_i->ri_list = &ch_ctx->rmt_rx_intent_list;
}
EXPORT_SYMBOL(glink_get_ch_intent_info);

/**
 * glink_get_debug_mask() - Return debug mask attribute
 *
 * Return: debug mask attribute
 */
unsigned glink_get_debug_mask(void)
{
	return glink_debug_mask;
}
EXPORT_SYMBOL(glink_get_debug_mask);

/**
 * glink_get_log_ctx() - Return log context for other GLINK modules.
 *
 * Return: Log context or NULL if none.
 */
void *glink_get_log_ctx(void)
{
	return log_ctx;
}
EXPORT_SYMBOL(glink_get_log_ctx);

static int glink_init(void)
{
	log_ctx = ipc_log_context_create(NUM_LOG_PAGES, "glink", 0);
	if (!log_ctx)
		GLINK_ERR("%s: unable to create log context\n", __func__);
	glink_debugfs_init();

	return 0;
}
module_init(glink_init);

MODULE_DESCRIPTION("MSM Generic Link (G-Link) Transport");
MODULE_LICENSE("GPL v2");
