/**
 * gadget.c - DesignWare USB3 DRD Controller Gadget Framework Link
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>

#include "core.h"
#include "gadget.h"
#include "debug.h"
#include "io.h"

#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
#include <linux/module.h>

#define EP0_HS_MPS 64
#define EP0_SS_MPS 512
#define WORK_CANCEL(udc)	cancel_work_sync(&udc->reconnect_work);
#define WORK_SCHEDULE(udc)	schedule_work(&udc->reconnect_work);

#else
#define WORK_CANCEL(udc)
#define WORK_SCHEDULE(udc)
#endif

/**
 * dwc3_gadget_set_test_mode - Enables USB2 Test Modes
 * @dwc: pointer to our context structure
 * @mode: the mode to set (J, K SE0 NAK, Force Enable)
 *
 * Caller should take care of locking. This function will
 * return 0 on success or -EINVAL if wrong Test Selector
 * is passed
 */
int dwc3_gadget_set_test_mode(struct dwc3 *dwc, int mode)
{
	u32		reg;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_TSTCTRL_MASK;

	switch (mode) {
	case TEST_J:
	case TEST_K:
	case TEST_SE0_NAK:
	case TEST_PACKET:
	case TEST_FORCE_EN:
		reg |= mode << 1;
		break;
	default:
		return -EINVAL;
	}

	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	return 0;
}

/**
 * dwc3_gadget_set_link_state - Sets USB Link to a particular State
 * @dwc: pointer to our context structure
 * @state: the state to put link into
 *
 * Caller should take care of locking. This function will
 * return 0 on success or -ETIMEDOUT.
 */
int dwc3_gadget_set_link_state(struct dwc3 *dwc, enum dwc3_link_state state)
{
	int		retries = 10000;
	u32		reg;

	/*
	 * Wait until device controller is ready. Only applies to 1.94a and
	 * later RTL.
	 */
	if (dwc->revision >= DWC3_REVISION_194A) {
		while (--retries) {
			reg = dwc3_readl(dwc->regs, DWC3_DSTS);
			if (reg & DWC3_DSTS_DCNRD)
				udelay(5);
			else
				break;
		}

		if (retries <= 0)
			return -ETIMEDOUT;
	}

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;

	/* set requested state */
	reg |= DWC3_DCTL_ULSTCHNGREQ(state);
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	/*
	 * The following code is racy when called from dwc3_gadget_wakeup,
	 * and is not needed, at least on newer versions
	 */
	if (dwc->revision >= DWC3_REVISION_194A)
		return 0;

	/* wait for a change in DSTS */
	retries = 10000;
	while (--retries) {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);

		if (DWC3_DSTS_USBLNKST(reg) == state)
			return 0;

		udelay(5);
	}

	dev_vdbg(dwc->dev, "link state change request timed out\n");

	return -ETIMEDOUT;
}

/**
 * dwc3_gadget_resize_tx_fifos - reallocate fifo spaces for current use-case
 * @dwc: pointer to our context structure
 *
 * This function will a best effort FIFO allocation in order
 * to improve FIFO usage and throughput, while still allowing
 * us to enable as many endpoints as possible.
 *
 * Keep in mind that this operation will be highly dependent
 * on the configured size for RAM1 - which contains TxFifo -,
 * the amount of endpoints enabled on coreConsultant tool, and
 * the width of the Master Bus.
 *
 * In the ideal world, we would always be able to satisfy the
 * following equation:
 *
 * ((512 + 2 * MDWIDTH-Bytes) + (Number of IN Endpoints - 1) * \
 * (3 * (1024 + MDWIDTH-Bytes) + MDWIDTH-Bytes)) / MDWIDTH-Bytes
 *
 * Unfortunately, due to many variables that's not always the case.
 */
int dwc3_gadget_resize_tx_fifos(struct dwc3 *dwc)
{
	int		last_fifo_depth = 0;
	int		ram1_depth;
	int		fifo_size;
	int		mdwidth;
	int		num;

	if (!dwc->needs_fifo_resize)
		return 0;

	ram1_depth = DWC3_RAM1_DEPTH(dwc->hwparams.hwparams7);
	mdwidth = DWC3_MDWIDTH(dwc->hwparams.hwparams0);

	/* MDWIDTH is represented in bits, we need it in bytes */
	mdwidth >>= 3;

	/*
	 * FIXME For now we will only allocate 1 wMaxPacketSize space
	 * for each enabled endpoint, later patches will come to
	 * improve this algorithm so that we better use the internal
	 * FIFO space. Also consider the case where TxFIFO RAM space
	 * may change dynamically based on the USB configuration.
	 */
	for (num = 0; num < DWC3_ENDPOINTS_NUM; num++) {
		struct dwc3_ep	*dep = dwc->eps[num];
		int		fifo_number = dep->number >> 1;
		int		mult = 1;
		int		tmp;

		if (!(dep->number & 1))
			continue;

		if (!(dep->flags & DWC3_EP_ENABLED))
			continue;

		if (((dep->endpoint.maxburst > 1) &&
				usb_endpoint_xfer_bulk(dep->endpoint.desc))
				|| usb_endpoint_xfer_isoc(dep->endpoint.desc))
			mult = 3;

		/*
		 * REVISIT: the following assumes we will always have enough
		 * space available on the FIFO RAM for all possible use cases.
		 * Make sure that's true somehow and change FIFO allocation
		 * accordingly.
		 *
		 * If we have Bulk (burst only) or Isochronous endpoints, we
		 * want them to be able to be very, very fast. So we're giving
		 * those endpoints a fifo_size which is enough for 3 full
		 * packets
		 */
		tmp = mult * (dep->endpoint.maxpacket + mdwidth);

		if (dwc->tx_fifo_size &&
			(usb_endpoint_xfer_bulk(dep->endpoint.desc)
			|| usb_endpoint_xfer_isoc(dep->endpoint.desc))) {
			/*
			 * Allocate 3KB fifo size for bulk and isochronous TX
			 * endpoints irrespective of speed if tx_fifo is not
			 * reduced. Otherwise allocate 1KB for endpoints in HS
			 * mode and for non burst endpoints in SS mode. For
			 * interrupt ep, allocate fifo size of ep maxpacket.
			 */
			if (!dwc->tx_fifo_reduced)
				tmp = 3 * (1024 + mdwidth);
			else
				tmp = mult * (1024 + mdwidth);
		}

		tmp += mdwidth;

		fifo_size = DIV_ROUND_UP(tmp, mdwidth);

		fifo_size |= (last_fifo_depth << 16);

		dev_vdbg(dwc->dev, "%s: Fifo Addr %04x Size %d\n",
				dep->name, last_fifo_depth, fifo_size & 0xffff);

		last_fifo_depth += (fifo_size & 0xffff);
		if (dwc->tx_fifo_size &&
				(last_fifo_depth >= dwc->tx_fifo_size)) {
			/*
			 * Fifo size allocated exceeded available RAM size.
			 * Hence return error.
			 */
			dev_err(dwc->dev, "Fifosize(%d) > available RAM(%d)\n",
					last_fifo_depth, dwc->tx_fifo_size);
			return -ENOMEM;
		}

		dwc3_writel(dwc->regs, DWC3_GTXFIFOSIZ(fifo_number),
				fifo_size);

	}

	return 0;
}

void dwc3_gadget_giveback(struct dwc3_ep *dep, struct dwc3_request *req,
		int status)
{
	struct dwc3			*dwc = dep->dwc;
	int				i;

	if (req->queued) {
		i = 0;
		do {
			dep->busy_slot++;
			/*
			 * Skip LINK TRB. We can't use req->trb and check for
			 * DWC3_TRBCTL_LINK_TRB because it points the TRB we
			 * just completed (not the LINK TRB).
			 */
			if (((dep->busy_slot & DWC3_TRB_MASK) ==
				DWC3_TRB_NUM- 1) &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc))
				dep->busy_slot++;
		} while(++i < req->request.num_mapped_sgs);
		req->queued = false;

		if (req->request.zero && req->ztrb) {
			dep->busy_slot++;
			req->ztrb = NULL;
			if (((dep->busy_slot & DWC3_TRB_MASK) ==
				DWC3_TRB_NUM - 1) &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc))
				dep->busy_slot++;
		}
	}
	list_del(&req->list);
	req->trb = NULL;

	if (req->request.status == -EINPROGRESS)
		req->request.status = status;

	if (dwc->ep0_bounced && dep->number == 0)
		dwc->ep0_bounced = false;
	else
		usb_gadget_unmap_request(&dwc->gadget, &req->request,
				req->direction);

	dev_dbg(dwc->dev, "request %p from %s completed %d/%d ===> %d\n",
			req, dep->name, req->request.actual,
			req->request.length, status);

	dbg_done(dep->number, req->request.actual, req->request.status);
	spin_unlock(&dwc->lock);
	if(req->request.complete)
		req->request.complete(&dep->endpoint, &req->request);
	spin_lock(&dwc->lock);
}

static const char *dwc3_gadget_ep_cmd_string(u8 cmd)
{
	switch (cmd) {
	case DWC3_DEPCMD_DEPSTARTCFG:
		return "Start New Configuration";
	case DWC3_DEPCMD_ENDTRANSFER:
		return "End Transfer";
	case DWC3_DEPCMD_UPDATETRANSFER:
		return "Update Transfer";
	case DWC3_DEPCMD_STARTTRANSFER:
		return "Start Transfer";
	case DWC3_DEPCMD_CLEARSTALL:
		return "Clear Stall";
	case DWC3_DEPCMD_SETSTALL:
		return "Set Stall";
	case DWC3_DEPCMD_GETEPSTATE:
		return "Get Endpoint State";
	case DWC3_DEPCMD_SETTRANSFRESOURCE:
		return "Set Endpoint Transfer Resource";
	case DWC3_DEPCMD_SETEPCONFIG:
		return "Set Endpoint Configuration";
	default:
		return "UNKNOWN command";
	}
}

int dwc3_send_gadget_generic_command(struct dwc3 *dwc, int cmd, u32 param)
{
	u32		timeout = 500;
	u32		reg;
	int ret;

	dwc3_writel(dwc->regs, DWC3_DGCMDPAR, param);
	dwc3_writel(dwc->regs, DWC3_DGCMD, cmd | DWC3_DGCMD_CMDACT);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DGCMD);
		if (!(reg & DWC3_DGCMD_CMDACT)) {
			dev_vdbg(dwc->dev, "Command Complete --> %d\n",
					DWC3_DGCMD_STATUS(reg));
			ret = 0;
			if (DWC3_DGCMD_STATUS(reg))
				return -EINVAL;
			break;
		}

		/*
		 * We can't sleep here, because it's also called from
		 * interrupt context.
		 */
		timeout--;
		if (!timeout) {
			ret = -ETIMEDOUT;
			break;
		}
		udelay(1);
	} while (1);

	return ret;
}

int dwc3_send_gadget_ep_cmd(struct dwc3 *dwc, unsigned ep,
		unsigned cmd, struct dwc3_gadget_ep_cmd_params *params)
{
	struct dwc3_ep		*dep = dwc->eps[ep];
	u32			timeout = 1500;
	u32			reg;
	int ret;

	dev_vdbg(dwc->dev, "%s: cmd '%s' params %08x %08x %08x\n",
			dep->name,
			dwc3_gadget_ep_cmd_string(cmd), params->param0,
			params->param1, params->param2);

	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR0(ep), params->param0);
	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR1(ep), params->param1);
	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR2(ep), params->param2);

	dwc3_writel(dwc->regs, DWC3_DEPCMD(ep), cmd | DWC3_DEPCMD_CMDACT);
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DEPCMD(ep));
		if (!(reg & DWC3_DEPCMD_CMDACT)) {
			dev_vdbg(dwc->dev, "Command Complete --> %d\n",
					DWC3_DEPCMD_STATUS(reg));
			if (DWC3_DEPCMD_STATUS(reg))
				return -EINVAL;
			/* SW issues START TRANSFER command to isochronous ep
			 * with future frame interval. If future interval time
			 * has already passed when core recieves command, core
			 * will respond with an error(bit13 in Command complete
			 * event. Hence return error in this case.
			 */
			if (reg & 0x2000)
				ret = -EAGAIN;
			else if (DWC3_DEPCMD_STATUS(reg))
				ret = -EINVAL;
			else
				ret = 0;
			break;
		}

		/*
		 * We can't sleep here, because it is also called from
		 * interrupt context.
		 */
		timeout--;
		if (!timeout) {
			ret = -ETIMEDOUT;
			break;
		}

		udelay(1);
	} while (1);

	return ret;
}

dma_addr_t dwc3_trb_dma_offset(struct dwc3_ep *dep,
		struct dwc3_trb *trb)
{
	u32		offset = (char *) trb - (char *) dep->trb_pool;

	return dep->trb_pool_dma + offset;
}

static int dwc3_alloc_trb_pool(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;

	if (dep->trb_pool)
		return 0;

	if (dep->number == 0 || dep->number == 1)
		return 0;

	dep->trb_pool = dma_alloc_coherent(dwc->dev,
			sizeof(struct dwc3_trb) * DWC3_TRB_NUM,
			&dep->trb_pool_dma, GFP_KERNEL);
	if (!dep->trb_pool) {
		dev_err(dep->dwc->dev, "failed to allocate trb pool for %s\n",
				dep->name);
		return -ENOMEM;
	}

	return 0;
}

static void dwc3_free_trb_pool(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;

	dma_free_coherent(dwc->dev, sizeof(struct dwc3_trb) * DWC3_TRB_NUM,
			dep->trb_pool, dep->trb_pool_dma);

	dep->trb_pool = NULL;
	dep->trb_pool_dma = 0;
}

static int dwc3_gadget_start_config(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_gadget_ep_cmd_params params;
	u32			cmd;

	memset(&params, 0x00, sizeof(params));

	if (dep->number != 1) {
		cmd = DWC3_DEPCMD_DEPSTARTCFG;
		/* XferRscIdx == 0 for ep0 and 2 for the remaining */
		if (dep->number > 1) {
			if (dwc->start_config_issued)
				return 0;
			dwc->start_config_issued = true;
			cmd |= DWC3_DEPCMD_PARAM(2);
		}

		return dwc3_send_gadget_ep_cmd(dwc, 0, cmd, &params);
	}

	return 0;
}

static int dwc3_gadget_set_ep_config(struct dwc3 *dwc, struct dwc3_ep *dep,
		const struct usb_endpoint_descriptor *desc,
		const struct usb_ss_ep_comp_descriptor *comp_desc,
		bool ignore)
{
	struct dwc3_gadget_ep_cmd_params params;

	memset(&params, 0x00, sizeof(params));

	params.param0 = DWC3_DEPCFG_EP_TYPE(usb_endpoint_type(desc))
		| DWC3_DEPCFG_MAX_PACKET_SIZE(usb_endpoint_maxp(desc));

	/* Burst size is only needed in SuperSpeed mode */
	if (dwc->gadget.speed == USB_SPEED_SUPER) {
		u32 burst = dep->endpoint.maxburst - 1;

		params.param0 |= DWC3_DEPCFG_BURST_SIZE(burst);
	}

	if (ignore)
		params.param0 |= DWC3_DEPCFG_IGN_SEQ_NUM;

	params.param1 = DWC3_DEPCFG_XFER_COMPLETE_EN
		| DWC3_DEPCFG_XFER_NOT_READY_EN;

	if (usb_ss_max_streams(comp_desc) && usb_endpoint_xfer_bulk(desc)) {
		params.param1 |= DWC3_DEPCFG_STREAM_CAPABLE
			| DWC3_DEPCFG_STREAM_EVENT_EN;
		dep->stream_capable = true;
	}

	if (usb_endpoint_xfer_isoc(desc))
		params.param1 |= DWC3_DEPCFG_XFER_IN_PROGRESS_EN;

	/*
	 * We are doing 1:1 mapping for endpoints, meaning
	 * Physical Endpoints 2 maps to Logical Endpoint 2 and
	 * so on. We consider the direction bit as part of the physical
	 * endpoint number. So USB endpoint 0x81 is 0x03.
	 */
	params.param1 |= DWC3_DEPCFG_EP_NUMBER(dep->number);

	/*
	 * We must use the lower 16 TX FIFOs even though
	 * HW might have more
	 */
	if (dep->direction)
		params.param0 |= DWC3_DEPCFG_FIFO_NUMBER(dep->number >> 1);

	if (desc->bInterval) {
		params.param1 |= DWC3_DEPCFG_BINTERVAL_M1(desc->bInterval - 1);
		dep->interval = 1 << (desc->bInterval - 1);
	}

	return dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETEPCONFIG, &params);
}

static int dwc3_gadget_set_xfer_resource(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_gadget_ep_cmd_params params;

	memset(&params, 0x00, sizeof(params));

	params.param0 = DWC3_DEPXFERCFG_NUM_XFER_RES(1);

	return dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETTRANSFRESOURCE, &params);
}

/**
 * __dwc3_gadget_ep_enable - Initializes a HW endpoint
 * @dep: endpoint to be initialized
 * @desc: USB Endpoint Descriptor
 *
 * Caller should take care of locking
 */
static int __dwc3_gadget_ep_enable(struct dwc3_ep *dep,
		const struct usb_endpoint_descriptor *desc,
		const struct usb_ss_ep_comp_descriptor *comp_desc,
		bool ignore)
{
	struct dwc3		*dwc = dep->dwc;
	u32			reg;
	int			ret = -ENOMEM;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		ret = dwc3_gadget_start_config(dwc, dep);
		if (ret)
			return ret;
	}

	ret = dwc3_gadget_set_ep_config(dwc, dep, desc, comp_desc, ignore);
	if (ret)
		return ret;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		struct dwc3_trb	*trb_st_hw;
		struct dwc3_trb	*trb_link;

		ret = dwc3_gadget_set_xfer_resource(dwc, dep);
		if (ret)
			return ret;

		dep->endpoint.desc = desc;
		dep->comp_desc = comp_desc;
		dep->type = usb_endpoint_type(desc);
		dep->flags |= DWC3_EP_ENABLED;

		reg = dwc3_readl(dwc->regs, DWC3_DALEPENA);
		reg |= DWC3_DALEPENA_EP(dep->number);
		dwc3_writel(dwc->regs, DWC3_DALEPENA, reg);

		if (!usb_endpoint_xfer_isoc(desc))
			return 0;

		/* Link TRB for ISOC. The HWO bit is never reset */
		trb_st_hw = &dep->trb_pool[0];

		trb_link = &dep->trb_pool[DWC3_TRB_NUM - 1];
		memset(trb_link, 0, sizeof(*trb_link));

		trb_link->bpl = lower_32_bits(dwc3_trb_dma_offset(dep, trb_st_hw));
		trb_link->bph = upper_32_bits(dwc3_trb_dma_offset(dep, trb_st_hw));
		trb_link->ctrl |= DWC3_TRBCTL_LINK_TRB;
		trb_link->ctrl |= DWC3_TRB_CTRL_HWO;
	}

	return 0;
}

static void dwc3_stop_active_transfer(struct dwc3 *dwc, u32 epnum);
static void dwc3_remove_requests(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_request		*req;

	if (!list_empty(&dep->req_queued)) {
		dwc3_stop_active_transfer(dwc, dep->number);

		/* - giveback all requests to gadget driver */
		while (!list_empty(&dep->req_queued)) {
			req = next_request(&dep->req_queued);

			dwc3_gadget_giveback(dep, req, -ESHUTDOWN);
		}
	}

	while (!list_empty(&dep->request_list)) {
		req = next_request(&dep->request_list);

		dwc3_gadget_giveback(dep, req, -ESHUTDOWN);
	}
}

/**
 * __dwc3_gadget_ep_disable - Disables a HW endpoint
 * @dep: the endpoint to disable
 *
 * This function also removes requests which are currently processed ny the
 * hardware and those which are not yet scheduled.
 * Caller should take care of locking.
 */
static int __dwc3_gadget_ep_disable(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;
	u32			reg;

	dwc3_remove_requests(dwc, dep);

	/* make sure HW endpoint isn't stalled */
	if (dep->flags & DWC3_EP_STALL)
		__dwc3_gadget_ep_set_halt(dep, 0, false);

	reg = dwc3_readl(dwc->regs, DWC3_DALEPENA);
	reg &= ~DWC3_DALEPENA_EP(dep->number);
	dwc3_writel(dwc->regs, DWC3_DALEPENA, reg);

	dep->stream_capable = false;
	dep->endpoint.desc = NULL;
	dep->comp_desc = NULL;
	dep->type = 0;
	dep->flags = 0;

	return 0;
}

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_ep0_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	return -EINVAL;
}

static int dwc3_gadget_ep0_disable(struct usb_ep *ep)
{
	return -EINVAL;
}

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long			flags;
	int				ret;

	if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		pr_debug("dwc3: invalid parameters\n");
		return -EINVAL;
	}

	if (!desc->wMaxPacketSize) {
		pr_debug("dwc3: missing wMaxPacketSize\n");
		return -EINVAL;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;

	if (dep->flags & DWC3_EP_ENABLED) {
		dev_WARN_ONCE(dwc->dev, true, "%s is already enabled\n",
				dep->name);
		return 0;
	}

	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		strlcat(dep->name, "-control", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_ISOC:
		strlcat(dep->name, "-isoc", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_BULK:
		strlcat(dep->name, "-bulk", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_INT:
		strlcat(dep->name, "-int", sizeof(dep->name));
		break;
	default:
		dev_err(dwc->dev, "invalid endpoint transfer type\n");
	}

	dev_vdbg(dwc->dev, "Enabling %s\n", dep->name);

	spin_lock_irqsave(&dwc->lock, flags);
	ret = __dwc3_gadget_ep_enable(dep, desc, ep->comp_desc, false);
	dbg_event(dep->number, "ENABLE", ret);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_disable(struct usb_ep *ep)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long			flags;
	int				ret;

	if (!ep) {
		pr_debug("dwc3: invalid parameters\n");
		return -EINVAL;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		dev_WARN_ONCE(dwc->dev, true, "%s is already disabled\n",
				dep->name);
		return 0;
	}

	snprintf(dep->name, sizeof(dep->name), "ep%d%s",
			dep->number >> 1,
			(dep->number & 1) ? "in" : "out");

	spin_lock_irqsave(&dwc->lock, flags);
	ret = __dwc3_gadget_ep_disable(dep);
	dbg_event(dep->number, "DISABLE", ret);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static struct usb_request *dwc3_gadget_ep_alloc_request(struct usb_ep *ep,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req;
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req) {
		dev_err(dwc->dev, "not enough memory\n");
		return NULL;
	}

	req->epnum	= dep->number;
	req->dep	= dep;

	return &req->request;
}

static void dwc3_gadget_ep_free_request(struct usb_ep *ep,
		struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);

	kfree(req);
}

/**
 * dwc3_prepare_one_trb - setup one TRB from one request
 * @dep: endpoint for which this request is prepared
 * @req: dwc3_request pointer
 */
static void dwc3_prepare_one_trb(struct dwc3_ep *dep,
		struct dwc3_request *req, dma_addr_t dma,
		unsigned length, unsigned last, unsigned chain, unsigned node)
{
	struct dwc3		*dwc = dep->dwc;
	struct dwc3_trb		*trb;
	bool			zlp_appended = false;
	unsigned		rlen;

	dev_vdbg(dwc->dev, "%s: req %p dma %08llx length %d%s%s\n",
			dep->name, req, (unsigned long long) dma,
			length, last ? " last" : "",
			chain ? " chain" : "");

	/* Skip the LINK-TRB on ISOC */
	if (((dep->free_slot & DWC3_TRB_MASK) == DWC3_TRB_NUM - 1) &&
			usb_endpoint_xfer_isoc(dep->endpoint.desc))
		dep->free_slot++;

	trb = &dep->trb_pool[dep->free_slot & DWC3_TRB_MASK];

	if (!req->trb) {
		dwc3_gadget_move_request_queued(req);
		req->trb = trb;
		req->trb_dma = dwc3_trb_dma_offset(dep, trb);
		req->start_slot = dep->free_slot & DWC3_TRB_MASK;
	}

	dep->free_slot++;

update_trb:
	trb->size = DWC3_TRB_SIZE_LENGTH(length);
	trb->bpl = lower_32_bits(dma);
	trb->bph = upper_32_bits(dma);

	switch (usb_endpoint_type(dep->endpoint.desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		trb->ctrl = DWC3_TRBCTL_CONTROL_SETUP;
		break;

	case USB_ENDPOINT_XFER_ISOC:
		if (!node)
			trb->ctrl = DWC3_TRBCTL_ISOCHRONOUS_FIRST;
		else
			trb->ctrl = DWC3_TRBCTL_ISOCHRONOUS;

		if (!req->request.no_interrupt && !chain)
			trb->ctrl |= DWC3_TRB_CTRL_IOC;
		break;

	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		trb->ctrl = DWC3_TRBCTL_NORMAL;
		break;
	default:
		/*
		 * This is only possible with faulty memory because we
		 * checked it already :)
		 */
		BUG();
	}

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		trb->ctrl |= DWC3_TRB_CTRL_ISP_IMI;
		trb->ctrl |= DWC3_TRB_CTRL_CSP;
	}

	if (chain)
		trb->ctrl |= DWC3_TRB_CTRL_CHN;

	if (usb_endpoint_xfer_bulk(dep->endpoint.desc) && dep->stream_capable)
		trb->ctrl |= DWC3_TRB_CTRL_SID_SOFN(req->request.stream_id);

	trb->ctrl |= DWC3_TRB_CTRL_HWO;

	rlen = req->request.length;
	if (!zlp_appended && !chain &&
		req->request.zero && rlen &&
		(rlen % usb_endpoint_maxp(dep->endpoint.desc) == 0)) {

		zlp_appended = true;
		/* Skip the LINK-TRB on ISOC */
		if (((dep->free_slot & DWC3_TRB_MASK) == DWC3_TRB_NUM - 1) &&
			usb_endpoint_xfer_isoc(dep->endpoint.desc))
			dep->free_slot++;

		trb->ctrl |= DWC3_TRB_CTRL_CHN;
		trb = &dep->trb_pool[dep->free_slot & DWC3_TRB_MASK];
		dep->free_slot++;

		req->ztrb = trb;
		length = 0;

		goto update_trb;
	}

	if (!usb_endpoint_xfer_isoc(dep->endpoint.desc) && last)
		trb->ctrl |= DWC3_TRB_CTRL_LST;
}

/*
 * dwc3_prepare_trbs - setup TRBs from requests
 * @dep: endpoint for which requests are being prepared
 * @starting: true if the endpoint is idle and no requests are queued.
 *
 * The function goes through the requests list and sets up TRBs for the
 * transfers. The function returns once there are no more TRBs available or
 * it runs out of requests.
 */
static void dwc3_prepare_trbs(struct dwc3_ep *dep, bool starting)
{
	struct dwc3_request	*req, *n;
	u32			trbs_left;
	u32			max;
	unsigned int		last_one = 0;
	int			maxpkt_size;
	bool			isoc;

	maxpkt_size = usb_endpoint_maxp(dep->endpoint.desc);
	isoc = usb_endpoint_xfer_isoc(dep->endpoint.desc);

	BUILD_BUG_ON_NOT_POWER_OF_2(DWC3_TRB_NUM);

	/* the first request must not be queued */
	trbs_left = (dep->busy_slot - dep->free_slot) & DWC3_TRB_MASK;

	/* Can't wrap around on a non-isoc EP since there's no link TRB */
	if (!usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		max = DWC3_TRB_NUM - (dep->free_slot & DWC3_TRB_MASK);
		if (trbs_left > max)
			trbs_left = max;
	}

	/*
	 * If busy & slot are equal than it is either full or empty. If we are
	 * starting to process requests then we are empty. Otherwise we are
	 * full and don't do anything
	 */
	if (!trbs_left) {
		if (!starting)
			return;
		trbs_left = DWC3_TRB_NUM;
		/*
		 * In case we start from scratch, we queue the ISOC requests
		 * starting from slot 1. This is done because we use ring
		 * buffer and have no LST bit to stop us. Instead, we place
		 * IOC bit every TRB_NUM/4. We try to avoid having an interrupt
		 * after the first request so we start at slot 1 and have
		 * 7 requests proceed before we hit the first IOC.
		 * Other transfer types don't use the ring buffer and are
		 * processed from the first TRB until the last one. Since we
		 * don't wrap around we have to start at the beginning.
		 */
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dep->busy_slot = 1;
			dep->free_slot = 1;
		} else {
			dep->busy_slot = 0;
			dep->free_slot = 0;
		}
	}

	list_for_each_entry_safe(req, n, &dep->request_list, list) {
		unsigned	length;
		dma_addr_t	dma;
		bool		last_req = list_is_last(&req->list,
							&dep->request_list);
		int		num_trbs_required = 0;

		last_one = false;

		/* The last TRB is a link TRB, not used for xfer */
		if (isoc)
			num_trbs_required++;

		if (req->request.num_mapped_sgs)
			num_trbs_required += req->request.num_mapped_sgs;
		else
			num_trbs_required++;

		if (req->request.zero && req->request.length &&
				(req->request.length % maxpkt_size == 0))
			num_trbs_required++;

		if (trbs_left < num_trbs_required)
			break;

		if (req->request.num_mapped_sgs > 0) {
			struct usb_request *request = &req->request;
			struct scatterlist *sg = request->sg;
			struct scatterlist *s;
			int		i;

			for_each_sg(sg, s, request->num_mapped_sgs, i) {
				unsigned chain = true;

				length = sg_dma_len(s);
				dma = sg_dma_address(s);

				if (i == (request->num_mapped_sgs - 1) ||
						sg_is_last(s)) {
					unsigned temp = 0;
					unsigned len;
					struct dwc3_request *nreq = n;
					struct usb_request *ureq;
					bool mpkt = false;

					chain = false;
					if (last_req) {
						last_one = true;
						goto start_trb_queuing;
					}

					/*
					 * check if trbs sufficient for next
					 * request to set the last bit
					 */
					ureq = &nreq->request;
					len  = ureq->length;

					if (len % maxpkt_size == 0)
						mpkt = true;

					if (ureq->zero && len && mpkt)
						temp++;

					if (ureq->num_mapped_sgs)
						temp +=
						ureq->num_mapped_sgs;
					else
						temp++;

					if (isoc)
						temp++;

					if (trbs_left <= temp)
						last_one = true;
				}

start_trb_queuing:
				trbs_left--;
				if (!trbs_left)
					last_one = true;

				if (last_one)
					chain = false;

				dwc3_prepare_one_trb(dep, req, dma, length,
						last_one, chain, i);

				if (last_one)
					break;
			}
			dbg_queue(dep->number, &req->request, trbs_left);
			if (last_one)
				break;
		} else {
			struct dwc3_request	*req1;
			int maxpkt_size = usb_endpoint_maxp(dep->endpoint.desc);

			dma = req->request.dma;
			length = req->request.length;
			trbs_left--;

			if (req->request.zero && length &&
						(length % maxpkt_size == 0))
				trbs_left--;

			if (!trbs_left) {
				last_one = 1;
			} else if (dep->direction && (trbs_left <= 1)) {
				req1 = next_request(&req->list);
				if (req1->request.zero && req1->request.length
				 && (req1->request.length % maxpkt_size == 0))
					last_one = 1;
			}

			/* Is this the last request? */
			if (last_req)
				last_one = 1;

			if((usb_endpoint_dir_in(dep->endpoint.desc)) ||
					(dep->dwc->gadget.sg_supported == true))
				dwc3_prepare_one_trb(dep, req, dma, length,
						last_one, false, 0);
			else if (dep->dwc->gadget.sg_supported == false) /* Only for NCM protocol */
				dwc3_prepare_one_trb(dep, req, dma, length,
						1, false, 0);

			dbg_queue(dep->number, &req->request, 0);
			if (last_one)
				break;
		}
	}
}

static int __dwc3_gadget_kick_transfer(struct dwc3_ep *dep, u16 cmd_param,
		int start_new)
{
	struct dwc3_gadget_ep_cmd_params params;
	struct dwc3_request		*req, *req1, *n;
	struct dwc3			*dwc = dep->dwc;
	int				ret;
	u32				cmd;

	if (start_new && (dep->flags & DWC3_EP_BUSY)) {
		dev_vdbg(dwc->dev, "%s: endpoint busy\n", dep->name);
		return -EBUSY;
	}
	dep->flags &= ~DWC3_EP_PENDING_REQUEST;

	/*
	 * If we are getting here after a short-out-packet we don't enqueue any
	 * new requests as we try to set the IOC bit only on the last request.
	 */
	if (start_new) {
		if (list_empty(&dep->req_queued))
			dwc3_prepare_trbs(dep, start_new);

		/* req points to the first request which will be sent */
		req = next_request(&dep->req_queued);
	} else {
		dwc3_prepare_trbs(dep, start_new);

		/*
		 * req points to the first request where HWO changed from 0 to 1
		 */
		req = next_request(&dep->req_queued);
	}
	if (!req) {
		dep->flags |= DWC3_EP_PENDING_REQUEST;
		dbg_event(dep->number, "NO REQ", 0);
		return 0;
	}

	memset(&params, 0, sizeof(params));

	if (start_new) {
		params.param0 = upper_32_bits(req->trb_dma);
		params.param1 = lower_32_bits(req->trb_dma);
		cmd = DWC3_DEPCMD_STARTTRANSFER;
	} else {
		cmd = DWC3_DEPCMD_UPDATETRANSFER;
	}

	cmd |= DWC3_DEPCMD_PARAM(cmd_param);
	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);
	if (ret < 0) {
		dev_dbg(dwc->dev, "failed to send STARTTRANSFER command\n");

		if ((ret == -EAGAIN) && start_new &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			/* If bit13 in Command complete event is set, software
			 * must issue ENDTRANDFER command and wait for
			 * Xfernotready event to queue the requests again.
			 */
			if (!dep->resource_index) {
				dep->resource_index =
					 dwc3_gadget_ep_get_transfer_index(dwc,
								dep->number);
				WARN_ON_ONCE(!dep->resource_index);
			}
			dwc3_stop_active_transfer(dwc, dep->number);
			list_for_each_entry_safe_reverse(req1, n,
						 &dep->req_queued, list) {
				req1->trb = NULL;
				dwc3_gadget_move_request_list_front(req1);
				if (req->request.num_mapped_sgs)
					dep->busy_slot +=
						 req->request.num_mapped_sgs;
				else
					dep->busy_slot++;
				if ((dep->busy_slot & DWC3_TRB_MASK) ==
							DWC3_TRB_NUM - 1)
					dep->busy_slot++;
			}
			return ret;
		} else {
			/*
			 * FIXME we need to iterate over the list of requests
			 * here and stop, unmap, free and del each of the linked
			 * requests instead of what we do now.
			 */
			usb_gadget_unmap_request(&dwc->gadget, &req->request,
				req->direction);
			list_del(&req->list);
			return ret;
		}
	}

	dep->flags |= DWC3_EP_BUSY;

	if (start_new) {
		dep->resource_index = dwc3_gadget_ep_get_transfer_index(dwc,
				dep->number);
		WARN_ON_ONCE(!dep->resource_index);
	}

	return 0;
}

static void __dwc3_gadget_start_isoc(struct dwc3 *dwc,
		struct dwc3_ep *dep, u32 cur_uf)
{
	u32 uf;
	int ret;

	dep->current_uf = cur_uf;

	if (list_empty(&dep->request_list)) {
		dev_vdbg(dwc->dev, "ISOC ep %s run out for requests.\n",
			dep->name);
		dep->flags |= DWC3_EP_PENDING_REQUEST;
		return;
	}

	/* 4 micro frames in the future */
	uf = cur_uf + dep->interval * 4;

	ret = __dwc3_gadget_kick_transfer(dep, uf, 1);
	if (ret < 0)
		dbg_event(dep->number, "QUEUE", ret);
}

static void dwc3_gadget_start_isoc(struct dwc3 *dwc,
		struct dwc3_ep *dep, const struct dwc3_event_depevt *event)
{
	u32 cur_uf, mask;

	mask = ~(dep->interval - 1);
	cur_uf = event->parameters & mask;

	__dwc3_gadget_start_isoc(dwc, dep, cur_uf);
}

static int __dwc3_gadget_ep_queue(struct dwc3_ep *dep, struct dwc3_request *req)
{
	struct dwc3		*dwc = dep->dwc;
	int			ret;

	if (req->request.status == -EINPROGRESS) {
		ret = -EBUSY;
		dev_err(dwc->dev, "%s: %p request already in queue",
					dep->name, req);
		return ret;
	}

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;
	req->direction		= dep->direction;
	req->epnum		= dep->number;

	/*
	 * We only add to our list of requests now and
	 * start consuming the list once we get XferNotReady
	 * IRQ.
	 *
	 * That way, we avoid doing anything that we don't need
	 * to do now and defer it until the point we receive a
	 * particular token from the Host side.
	 *
	 * This will also avoid Host cancelling URBs due to too
	 * many NAKs.
	 */
	ret = usb_gadget_map_request(&dwc->gadget, &req->request,
			dep->direction);
	if (ret)
		return ret;

	list_add_tail(&req->list, &dep->request_list);

	/*
	 * There are a few special cases:
	 *
	 * 1. XferNotReady with empty list of requests. We need to kick the
	 *    transfer here in that situation, otherwise we will be NAKing
	 *    forever. If we get XferNotReady before gadget driver has a
	 *    chance to queue a request, we will ACK the IRQ but won't be
	 *    able to receive the data until the next request is queued.
	 *    The following code is handling exactly that.
	 *
	 */
	if (dep->flags & DWC3_EP_PENDING_REQUEST) {
		/*
		 * If xfernotready is already elapsed and it is a case
		 * of isoc transfer, then issue END TRANSFER, so that
		 * you can receive xfernotready again and can have
		 * notion of current microframe.
		 */
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			/* If xfernotready event is recieved before issuing
			 * START TRANSFER command, don't issue END TRANSFER.
			 * Rather start queueing the requests by issuing START
			 * TRANSFER command.
			 */
			if (list_empty(&dep->req_queued) && dep->resource_index)
				dwc3_stop_active_transfer(dwc, dep->number);
			else
				__dwc3_gadget_start_isoc(dwc, dep,
							dep->current_uf);
			dep->flags &= ~DWC3_EP_PENDING_REQUEST;
			return 0;
		}

		ret = __dwc3_gadget_kick_transfer(dep, 0, true);
		if (ret && ret != -EBUSY) {
			dbg_event(dep->number, "QUEUE", ret);
			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}
		return ret;
	}

	/*
	 * 2. XferInProgress on Isoc EP with an active transfer. We need to
	 *    kick the transfer here after queuing a request, otherwise the
	 *    core may not see the modified TRB(s).
	 */
	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
			(dep->flags & DWC3_EP_BUSY) &&
			!(dep->flags & DWC3_EP_MISSED_ISOC)) {
		WARN_ON_ONCE(!dep->resource_index);
		ret = __dwc3_gadget_kick_transfer(dep, dep->resource_index,
				false);
		if (ret && ret != -EBUSY) {
			dbg_event(dep->number, "QUEUE", ret);
			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}
		return ret;
	}

	return 0;
}

static inline enum dwc3_link_state dwc3_get_link_state(struct dwc3 *dwc)
{
	u32 reg;
	reg = dwc3_readl(dwc->regs, DWC3_DSTS);
	return DWC3_DSTS_USBLNKST(reg);
}

static int dwc3_gadget_ep_queue(struct usb_ep *ep, struct usb_request *request,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;

	int				ret;

	spin_lock_irqsave(&dwc->lock, flags);

	if (!dep->endpoint.desc) {
		spin_unlock_irqrestore(&dwc->lock, flags);
		dev_dbg(dwc->dev, "trying to queue request %p to disabled %s\n",
				request, ep->name);
		return -ESHUTDOWN;
	}

	dev_vdbg(dwc->dev, "queing request %p to %s length %d\n",
			request, ep->name, request->length);

	WARN(!dep->direction && (request->length % ep->desc->wMaxPacketSize),
		"trying to queue unaligned request (%d)\n", request->length);

	ret = __dwc3_gadget_ep_queue(dep, req);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_dequeue(struct usb_ep *ep,
		struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_request		*r = NULL;

	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;
	int				ret = 0;

	spin_lock_irqsave(&dwc->lock, flags);

	list_for_each_entry(r, &dep->request_list, list) {
		if (r == req)
			break;
	}

	if (r != req) {
		list_for_each_entry(r, &dep->req_queued, list) {
			if (r == req)
				break;
		}
		if (r == req) {
			/* wait until it is processed */
			dwc3_stop_active_transfer(dwc, dep->number);
			goto out1;
		}
		dev_err(dwc->dev, "request %p was not queued to %s\n",
				request, ep->name);
		ret = -EINVAL;
		goto out0;
	}

out1:
	dbg_event(dep->number, "DEQUEUE", 0);
	/* giveback the request */
	dwc3_gadget_giveback(dep, req, -ECONNRESET);

out0:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

int __dwc3_gadget_ep_set_halt(struct dwc3_ep *dep, int value, int protocol)
{
	struct dwc3_gadget_ep_cmd_params	params;
	struct dwc3				*dwc = dep->dwc;
	int					ret;

	memset(&params, 0x00, sizeof(params));

	if (value) {
		if (!protocol && ((dep->direction && dep->flags & DWC3_EP_BUSY) ||
				(!list_empty(&dep->req_queued) ||
				 !list_empty(&dep->request_list)))) {
			dev_dbg(dwc->dev, "%s: pending request, cannot halt\n",
					dep->name);
			return -EAGAIN;
		}

		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETSTALL, &params);
		if (ret)
			dev_err(dwc->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags |= DWC3_EP_STALL;
	} else {
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_CLEARSTALL, &params);
		if (ret)
			dev_err(dwc->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags &= ~(DWC3_EP_STALL | DWC3_EP_WEDGE);
	}

	return ret;
}

static int dwc3_gadget_ep_set_halt(struct usb_ep *ep, int value)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;

	int				ret;

	spin_lock_irqsave(&dwc->lock, flags);

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		dev_err(dwc->dev, "%s is of Isochronous type\n", dep->name);
		ret = -EINVAL;
		goto out;
	}

	dbg_event(dep->number, "HALT", value);
	ret = __dwc3_gadget_ep_set_halt(dep, value, false);
out:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_set_wedge(struct usb_ep *ep)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;
	unsigned long			flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dbg_event(dep->number, "WEDGE", 0);
	dep->flags |= DWC3_EP_WEDGE;
	spin_unlock_irqrestore(&dwc->lock, flags);

	if (dep->number == 0 || dep->number == 1)
		return dwc3_gadget_ep0_set_halt(ep, 1);
	else
		return __dwc3_gadget_ep_set_halt(dep, 1, false);
}

/* -------------------------------------------------------------------------- */

static struct usb_endpoint_descriptor dwc3_gadget_ep0_desc = {
	.bLength	= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes	= USB_ENDPOINT_XFER_CONTROL,
};

static const struct usb_ep_ops dwc3_gadget_ep0_ops = {
	.enable		= dwc3_gadget_ep0_enable,
	.disable	= dwc3_gadget_ep0_disable,
	.alloc_request	= dwc3_gadget_ep_alloc_request,
	.free_request	= dwc3_gadget_ep_free_request,
	.queue		= dwc3_gadget_ep0_queue,
	.dequeue	= dwc3_gadget_ep_dequeue,
	.set_halt	= dwc3_gadget_ep0_set_halt,
	.set_wedge	= dwc3_gadget_ep_set_wedge,
};

static const struct usb_ep_ops dwc3_gadget_ep_ops = {
	.enable		= dwc3_gadget_ep_enable,
	.disable	= dwc3_gadget_ep_disable,
	.alloc_request	= dwc3_gadget_ep_alloc_request,
	.free_request	= dwc3_gadget_ep_free_request,
	.queue		= dwc3_gadget_ep_queue,
	.dequeue	= dwc3_gadget_ep_dequeue,
	.set_halt	= dwc3_gadget_ep_set_halt,
	.set_wedge	= dwc3_gadget_ep_set_wedge,
};

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_get_frame(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	u32			reg;

	reg = dwc3_readl(dwc->regs, DWC3_DSTS);
	return DWC3_DSTS_SOFFN(reg);
}

static int dwc3_gadget_wakeup(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);

	unsigned long		timeout;
	unsigned long		flags;

	u32			reg;

	int			ret = 0;

	u8			link_state;
	u8			speed;

	spin_lock_irqsave(&dwc->lock, flags);

	/*
	 * According to the Databook Remote wakeup request should
	 * be issued only when the device is in early suspend state.
	 *
	 * We can check that via USB Link State bits in DSTS register.
	 */
	reg = dwc3_readl(dwc->regs, DWC3_DSTS);

	speed = reg & DWC3_DSTS_CONNECTSPD;
	if (speed == DWC3_DSTS_SUPERSPEED) {
		dev_dbg(dwc->dev, "no wakeup on SuperSpeed\n");
		ret = -EINVAL;
		goto out;
	}

	link_state = DWC3_DSTS_USBLNKST(reg);

	switch (link_state) {
	case DWC3_LINK_STATE_RX_DET:	/* in HS, means Early Suspend */
	case DWC3_LINK_STATE_U3:	/* in HS, means SUSPEND */
		break;
	default:
		dev_dbg(dwc->dev, "can't wakeup from link state %d\n",
				link_state);
		ret = -EINVAL;
		goto out;
	}

	ret = dwc3_gadget_set_link_state(dwc, DWC3_LINK_STATE_RECOV);
	if (ret < 0) {
		dev_err(dwc->dev, "failed to put link in Recovery\n");
		goto out;
	}

	/* Recent versions do this automatically */
	if (dwc->revision < DWC3_REVISION_194A) {
		/* write zeroes to Link Change Request */
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	}

	/* poll until Link State changes to ON */
	timeout = jiffies + msecs_to_jiffies(100);

	while (!time_after(jiffies, timeout)) {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);

		/* in HS, means ON */
		if (DWC3_DSTS_USBLNKST(reg) == DWC3_LINK_STATE_U0)
			break;
	}

	if (DWC3_DSTS_USBLNKST(reg) != DWC3_LINK_STATE_U0) {
		dev_err(dwc->dev, "failed to send remote wakeup\n");
		ret = -EINVAL;
	}

out:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_set_selfpowered(struct usb_gadget *g,
		int is_selfpowered)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dwc->is_selfpowered = !!is_selfpowered;
	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

#define DWC3_SOFT_RESET_TIMEOUT	10  /* 10 msec */
static int dwc3_gadget_run_stop(struct dwc3 *dwc, int is_on)
{
	u32			reg;
	u32			timeout = 500;
	ktime_t start, diff;

	printk(KERN_DEBUG"usb: %s is_on:%d\n",__func__, is_on);

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	if (is_on) {
		if (dwc->revision <= DWC3_REVISION_187A) {
			reg &= ~DWC3_DCTL_TRGTULST_MASK;
			reg |= DWC3_DCTL_TRGTULST_RX_DET;
		}

		if (dwc->revision >= DWC3_REVISION_194A)
			reg &= ~DWC3_DCTL_KEEP_CONNECT;

		start = ktime_get();
		/* issue device SoftReset */
		dwc3_writel(dwc->regs, DWC3_DCTL, reg | DWC3_DCTL_CSFTRST);
		do {
			reg = dwc3_readl(dwc->regs, DWC3_DCTL);
			if (!(reg & DWC3_DCTL_CSFTRST))
				break;

			diff = ktime_sub(ktime_get(), start);
			/* poll for max. 10ms */
			if (ktime_to_ms(diff) > DWC3_SOFT_RESET_TIMEOUT) {
				printk_ratelimited(KERN_ERR
					"%s:core Reset Timed Out\n", __func__);
				break;
			}
			cpu_relax();
		} while (true);


		dwc3_event_buffers_setup(dwc);
		dwc3_gadget_restart(dwc);
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg |= DWC3_DCTL_RUN_STOP;
		dwc->pullups_connected = true;
	} else {
		reg &= ~DWC3_DCTL_RUN_STOP;
		dwc->pullups_connected = false;
	}

	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);
		if (is_on) {
			if (!(reg & DWC3_DSTS_DEVCTRLHLT))
				break;
		} else {
			if (reg & DWC3_DSTS_DEVCTRLHLT)
				break;
		}
		timeout--;
		if (!timeout)
			return -ETIMEDOUT;
		udelay(1);
	} while (1);

	dev_vdbg(dwc->dev, "gadget %s data soft-%s\n",
			dwc->gadget_driver
			? dwc->gadget_driver->function : "no-function",
			is_on ? "connect" : "disconnect");

	return 0;
}

static int dwc3_gadget_vbus_draw(struct usb_gadget *g, unsigned mA)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	struct dwc3_otg		*dotg = dwc->dotg;

	if (dotg && dotg->otg.phy)
		return usb_phy_set_power(dotg->otg.phy, mA);

	return -ENOTSUPP;
}

static int dwc3_gadget_pullup(struct usb_gadget *g, int is_on)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	int			ret;

	is_on = !!is_on;

	spin_lock_irqsave(&dwc->lock, flags);

	dwc->softconnect = is_on;

	if ((dwc->dotg && !dwc->vbus_active) ||
		!dwc->gadget_driver) {

		spin_unlock_irqrestore(&dwc->lock, flags);

		/*
		 * Need to wait for vbus_session(on) from otg driver or to
		 * the udc_start.
		 */
		return 0;
	}

	ret = dwc3_gadget_run_stop(dwc, is_on);

	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

void dwc3_gadget_enable_irq(struct dwc3 *dwc)
{
	u32			reg;

	/* Enable all but Start and End of Frame IRQs */
	reg = (DWC3_DEVTEN_EVNTOVERFLOWEN |
			DWC3_DEVTEN_CMDCMPLTEN |
			DWC3_DEVTEN_ERRTICERREN |
			DWC3_DEVTEN_WKUPEVTEN |
			DWC3_DEVTEN_CONNECTDONEEN |
			DWC3_DEVTEN_USBRSTEN |
			DWC3_DEVTEN_DISCONNEVTEN);

	/*
	 * Enable SUSPENDEVENT(BIT:6) for version 230A and above
	 * else enable USB Link change event (BIT:3) for older version
	 */
	if (dwc->revision < DWC3_REVISION_230A)
		reg |= DWC3_DEVTEN_ULSTCNGEN;
	else if (dwc->enable_suspend_event)
		reg |= DWC3_DEVTEN_SUSPEND;

	dwc3_writel(dwc->regs, DWC3_DEVTEN, reg);
}

void dwc3_gadget_disable_irq(struct dwc3 *dwc)
{
	/* mask all interrupts */
	dwc3_writel(dwc->regs, DWC3_DEVTEN, 0x00);
}

static irqreturn_t dwc3_interrupt(int irq, void *_dwc);
static irqreturn_t dwc3_thread_interrupt(int irq, void *_dwc);
static void dwc3_gadget_disconnect_interrupt(struct dwc3 *dwc);

static int dwc3_gadget_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	struct dwc3 *dwc = gadget_to_dwc(_gadget);
	unsigned long flags;
#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
	int cancel_work = 0;
#endif

	if (!dwc->dotg)
		return -EPERM;

	is_active = !!is_active;

	spin_lock_irqsave(&dwc->lock, flags);

#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
        if (!is_active)
                dwc->ss_host_avail = -1;
#endif

	/* Mark that the vbus was powered */
	dwc->vbus_active = is_active;

	/*
	 * Check if upper level usb_gadget_driver was already registerd with
	 * this udc controller driver (if dwc3_gadget_start was called)
	 */
	if (dwc->gadget_driver && dwc->softconnect) {
		if (dwc->vbus_active) {
			/*
			 * Both vbus was activated by otg and pullup was
			 * signaled by the gadget driver.
			 */
			dwc3_gadget_run_stop(dwc, 1);
		} else {
#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
			dwc->ss_host_avail = -1;
			dwc->speed_limit = dwc->gadget.max_speed;
			cancel_work = 1;
#endif
			dwc3_gadget_run_stop(dwc, 0);
		}
	}

	/*
	 * Clearing run/stop bit might occur before disconnect event is seen.
	 * Make sure to let gadget driver know in that case.
	 */
	if (!dwc->vbus_active) {
		pr_info("calling disconnect from %s\n", __func__);
		dwc3_gadget_disconnect_interrupt(dwc);
	}

	spin_unlock_irqrestore(&dwc->lock, flags);
#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
	if (cancel_work)
		WORK_CANCEL(dwc);
#endif
	return 0;
}

static int __dwc3_gadget_start(struct dwc3 *dwc)
{
	struct dwc3_ep		*dep;
	int			ret = 0;
	u32			reg;
#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
	int			ep0_mps;
#endif
	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg &= ~(DWC3_DCFG_SPEED_MASK);

	/**
	 * WORKAROUND: DWC3 revision < 2.20a have an issue
	 * which would cause metastability state on Run/Stop
	 * bit if we try to force the IP to USB2-only mode.
	 *
	 * Because of that, we cannot configure the IP to any
	 * speed other than the SuperSpeed
	 *
	 * Refers to:
	 *
	 * STAR#9000525659: Clock Domain Crossing on DCTL in
	 * USB 2.0 Mode
	 */
#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
	switch(dwc->speed_limit) {
	case USB_SPEED_SUPER:
		reg |= DWC3_DCFG_SUPERSPEED;
		break;
	case USB_SPEED_HIGH:
		reg |= DWC3_DCFG_HIGHSPEED;
		break;
	case USB_SPEED_FULL:
		reg |= DWC3_DCFG_FULLSPEED1;
		break;
	default:
		reg |= DWC3_DCFG_SUPERSPEED;
	}
#else
	if (dwc->revision < DWC3_REVISION_220A) {
		reg |= DWC3_DCFG_SUPERSPEED;
	} else {
		switch (dwc->maximum_speed) {
		case USB_SPEED_LOW:
			reg |= DWC3_DSTS_LOWSPEED;
			break;
		case USB_SPEED_FULL:
			reg |= DWC3_DSTS_FULLSPEED2;
			break;
		case USB_SPEED_HIGH:
			reg |= DWC3_DSTS_HIGHSPEED;
			break;
		case USB_SPEED_SUPER:	/* FALLTHROUGH */
		case USB_SPEED_UNKNOWN:	/* FALTHROUGH */
		default:
			reg |= DWC3_DSTS_SUPERSPEED;
		}
	}
#endif
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);

	dwc->start_config_issued = false;

#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
	switch (dwc->speed_limit) {
	case USB_SPEED_FULL:
	case USB_SPEED_HIGH:
		ep0_mps = EP0_HS_MPS;
		break;
	case USB_SPEED_SUPER:
		ep0_mps = EP0_SS_MPS;
		break;
	default:
		dev_warn(dwc->dev, "%s: unsupported device speed\n", __func__);
		ep0_mps = (dwc->gadget.max_speed == USB_SPEED_SUPER) ?
		EP0_SS_MPS : EP0_HS_MPS;
	}
	dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(ep0_mps);
#else

	/* Start with SuperSpeed Default */
	dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);
#endif	/* CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU*/

	dwc->delayed_status = false;
	/* reinitialize physical ep0-1 */
	dep = dwc->eps[0];
	dep->flags = 0;
	dep->endpoint.maxburst = 1;
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return ret;
	}

	dep = dwc->eps[1];
	dep->flags = 0;
	dep->endpoint.maxburst = 1;
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		__dwc3_gadget_ep_disable(dwc->eps[0]);
		return ret;
	}

	/* begin to receive SETUP packets */
	dwc->ep0state = EP0_SETUP_PHASE;
	dwc3_ep0_out_start(dwc);

	dwc3_gadget_enable_irq(dwc);

	return ret;
}

/* Required gadget re-initialization before switching to gadget in OTG mode */
void dwc3_gadget_restart(struct dwc3 *dwc)
{
	__dwc3_gadget_start(dwc);
}

static int dwc3_gadget_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	int			ret = 0;
	int			irq;

	pm_runtime_get_sync(dwc->dev);
	irq = platform_get_irq(to_platform_device(dwc->dev), 0);
	dwc->irq = irq;
	ret = request_irq(irq, dwc3_interrupt, IRQF_SHARED, "dwc3", dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to request irq #%d --> %d\n",
				irq, ret);
		goto err0;
	}

	spin_lock_irqsave(&dwc->lock, flags);

	if (dwc->gadget_driver) {
		dev_err(dwc->dev, "%s is already bound to %s\n",
				dwc->gadget.name,
				dwc->gadget_driver->driver.name);
		ret = -EBUSY;
		goto err1;
	}

	dwc->gadget_driver	= driver;

	/*
	 * For DRD, this might get called by gadget driver during bootup
	 * even though host mode might be active. Don't actually perform
	 * device-specific initialization until device mode is activated.
	 * In that case dwc3_gadget_restart() will handle it.
	 */
	if (!dwc->dotg) {
		ret = __dwc3_gadget_start(dwc);
		if (ret)
			goto err1;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);
	pm_runtime_put(dwc->dev);

	return 0;

err1:
	spin_unlock_irqrestore(&dwc->lock, flags);
	pm_runtime_put(dwc->dev);

err0:
	free_irq(irq, dwc);

	return ret;
}

static int dwc3_gadget_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	int			irq;

	dwc3_gadget_disable_irq(dwc);

	tasklet_kill(&dwc->bh);

	spin_lock_irqsave(&dwc->lock, flags);

	__dwc3_gadget_ep_disable(dwc->eps[0]);
	__dwc3_gadget_ep_disable(dwc->eps[1]);

	dwc->gadget_driver	= NULL;

	spin_unlock_irqrestore(&dwc->lock, flags);

	irq = platform_get_irq(to_platform_device(dwc->dev), 0);
	free_irq(irq, dwc);

	return 0;
}

static const struct usb_gadget_ops dwc3_gadget_ops = {
	.get_frame		= dwc3_gadget_get_frame,
	.wakeup			= dwc3_gadget_wakeup,
	.set_selfpowered	= dwc3_gadget_set_selfpowered,
	.vbus_session		= dwc3_gadget_vbus_session,
	.vbus_draw		= dwc3_gadget_vbus_draw,
	.pullup			= dwc3_gadget_pullup,
	.udc_start		= dwc3_gadget_start,
	.udc_stop		= dwc3_gadget_stop,
};

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_init_hw_endpoints(struct dwc3 *dwc,
		u8 num, u32 direction)
{
	struct dwc3_ep			*dep;
	u8				i;

	for (i = 0; i < num; i++) {
		u8 epnum = (i << 1) | (!!direction);

		dep = kzalloc(sizeof(*dep), GFP_KERNEL);
		if (!dep) {
			dev_err(dwc->dev, "can't allocate endpoint %d\n",
					epnum);
			return -ENOMEM;
		}

		dep->dwc = dwc;
		dep->number = epnum;
		dwc->eps[epnum] = dep;

		snprintf(dep->name, sizeof(dep->name), "ep%d%s", epnum >> 1,
				(epnum & 1) ? "in" : "out");

		dep->endpoint.name = dep->name;
		dep->direction = (epnum & 1);

		if (epnum == 0 || epnum == 1) {
			dep->endpoint.maxpacket = 512;
			dep->endpoint.maxburst = 1;
			dep->endpoint.ops = &dwc3_gadget_ep0_ops;
			if (!epnum)
				dwc->gadget.ep0 = &dep->endpoint;
		} else {
			int		ret;

			dep->endpoint.maxpacket = 1024;
			dep->endpoint.max_streams = 15;
			dep->endpoint.ops = &dwc3_gadget_ep_ops;
			list_add_tail(&dep->endpoint.ep_list,
					&dwc->gadget.ep_list);

			ret = dwc3_alloc_trb_pool(dep);
			if (ret)
				return ret;
		}

		INIT_LIST_HEAD(&dep->request_list);
		INIT_LIST_HEAD(&dep->req_queued);
	}

	return 0;
}

static int dwc3_gadget_init_endpoints(struct dwc3 *dwc)
{
	int				ret;

	INIT_LIST_HEAD(&dwc->gadget.ep_list);

	ret = dwc3_gadget_init_hw_endpoints(dwc, dwc->num_out_eps, 0);
	if (ret < 0) {
		dev_vdbg(dwc->dev, "failed to allocate OUT endpoints\n");
		return ret;
	}

	ret = dwc3_gadget_init_hw_endpoints(dwc, dwc->num_in_eps, 1);
	if (ret < 0) {
		dev_vdbg(dwc->dev, "failed to allocate IN endpoints\n");
		return ret;
	}

	return 0;
}

static void dwc3_gadget_free_endpoints(struct dwc3 *dwc)
{
	struct dwc3_ep			*dep;
	u8				epnum;

	for (epnum = 0; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		dep = dwc->eps[epnum];
		if (!dep)
			continue;
		/*
		 * Physical endpoints 0 and 1 are special; they form the
		 * bi-directional USB endpoint 0.
		 *
		 * For those two physical endpoints, we don't allocate a TRB
		 * pool nor do we add them the endpoints list. Due to that, we
		 * shouldn't do these two operations otherwise we would end up
		 * with all sorts of bugs when removing dwc3.ko.
		 */
		if (epnum != 0 && epnum != 1) {
			dwc3_free_trb_pool(dep);
			list_del(&dep->endpoint.ep_list);
		}

		kfree(dep);
	}
}

/* -------------------------------------------------------------------------- */

static int __dwc3_cleanup_done_trbs(struct dwc3 *dwc, struct dwc3_ep *dep,
		struct dwc3_request *req, struct dwc3_trb *trb,
		const struct dwc3_event_depevt *event, int status)
{
	unsigned int		count;
	unsigned int		s_pkt = 0;
	unsigned int		trb_status;

	if ((trb->ctrl & DWC3_TRB_CTRL_HWO) && status != -ESHUTDOWN)
		/*
		 * We continue despite the error. There is not much we
		 * can do. If we don't clean it up we loop forever. If
		 * we skip the TRB then it gets overwritten after a
		 * while since we use them in a ring buffer. A BUG()
		 * would help. Lets hope that if this occurs, someone
		 * fixes the root cause instead of looking away :)
		 */
		dev_err(dwc->dev, "%s's TRB (%p) still owned by HW\n",
				dep->name, trb);
	count = trb->size & DWC3_TRB_SIZE_MASK;

	if (dep->direction) {
		if (count) {
			trb_status = DWC3_TRB_SIZE_TRBSTS(trb->size);
			if (trb_status == DWC3_TRBSTS_MISSED_ISOC) {
				dev_dbg(dwc->dev, "incomplete IN transfer %s\n",
						dep->name);
				/*
				 * If missed isoc occurred and there is
				 * no request queued then issue END
				 * TRANSFER, so that core generates
				 * next xfernotready and we will issue
				 * a fresh START TRANSFER.
				 * If there are still queued request
				 * then wait, do not issue either END
				 * or UPDATE TRANSFER, just attach next
				 * request in request_list during
				 * giveback.If any future queued request
				 * is successfully transferred then we
				 * will issue UPDATE TRANSFER for all
				 * request in the request_list.
				 */
				dep->flags |= DWC3_EP_MISSED_ISOC;
				dbg_event(dep->number, "MISSED ISOC", status);
			} else {
				dev_err(dwc->dev, "incomplete IN transfer %s\n",
						dep->name);
				status = -ECONNRESET;
			}
		} else {
			dep->flags &= ~DWC3_EP_MISSED_ISOC;
		}
	} else {
		if (count && (event->status & DEPEVT_STATUS_SHORT))
			s_pkt = 1;
	}

	/*
	 * We assume here we will always receive the entire data block
	 * which we should receive. Meaning, if we program RX to
	 * receive 4K but we receive only 2K, we assume that's all we
	 * should receive and we simply bounce the request back to the
	 * gadget driver for further processing.
	 */
	req->request.actual += req->request.length - count;
	if (s_pkt)
		return 1;
	if ((event->status & DEPEVT_STATUS_LST) &&
			(trb->ctrl & (DWC3_TRB_CTRL_LST |
				DWC3_TRB_CTRL_HWO)))
		return 1;
	if ((event->status & DEPEVT_STATUS_IOC) &&
			(trb->ctrl & DWC3_TRB_CTRL_IOC))
		return 1;
	return 0;
}

static int dwc3_cleanup_done_reqs(struct dwc3 *dwc, struct dwc3_ep *dep,
		const struct dwc3_event_depevt *event, int status)
{
	struct dwc3_request	*req;
	struct dwc3_trb		*trb;
	unsigned int		slot;
	unsigned int		i;
	int			ret;

	do {
		req = next_request(&dep->req_queued);
		if (!req) {
			WARN_ON_ONCE(1);
			return 1;
		}
		i = 0;
		do {
			slot = req->start_slot + i;
			if ((slot == DWC3_TRB_NUM - 1) &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc))
				slot++;
			slot %= DWC3_TRB_NUM;
			trb = &dep->trb_pool[slot];

			ret = __dwc3_cleanup_done_trbs(dwc, dep, req, trb,
					event, status);
			if (ret)
				break;
		}while (++i < req->request.num_mapped_sgs);

		if (req->ztrb) {
			trb = req->ztrb;
			if ((event->status & DEPEVT_STATUS_LST) &&
				(trb->ctrl & (DWC3_TRB_CTRL_LST |
					DWC3_TRB_CTRL_HWO)))
				ret = 1;

			if ((event->status & DEPEVT_STATUS_IOC) &&
					(trb->ctrl & DWC3_TRB_CTRL_IOC))
				ret = 1;
		}
		dwc3_gadget_giveback(dep, req, status);

		/* EP possibly disabled during giveback? */
		if (!(dep->flags & DWC3_EP_ENABLED)) {
			dev_dbg(dwc->dev, "%s disabled while handling ep event\n",
					dep->name);
			return 0;
		}

		if (ret)
			break;
	} while (1);

	dwc->gadget.xfer_isr_count++;

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
			list_empty(&dep->req_queued)) {
		if (list_empty(&dep->request_list))
			/*
			 * If there is no entry in request list then do
			 * not issue END TRANSFER now. Just set PENDING
			 * flag, so that END TRANSFER is issued when an
			 * entry is added into request list.
			 */
			dep->flags |= DWC3_EP_PENDING_REQUEST;
		else
			dwc3_stop_active_transfer(dwc, dep->number);
		dep->flags &= ~DWC3_EP_MISSED_ISOC;
		return 1;
	}

	if ((event->status & DEPEVT_STATUS_IOC) &&
			(trb->ctrl & DWC3_TRB_CTRL_IOC))
		return 0;
	return 1;
}

static void dwc3_endpoint_transfer_complete(struct dwc3 *dwc,
		struct dwc3_ep *dep, const struct dwc3_event_depevt *event,
		int start_new)
{
	unsigned		status = 0;
	int			clean_busy;

	if (event->status & DEPEVT_STATUS_BUSERR)
		status = -ECONNRESET;

	clean_busy = dwc3_cleanup_done_reqs(dwc, dep, event, status);
	if (clean_busy)
		dep->flags &= ~DWC3_EP_BUSY;

	/*
	 * WORKAROUND: This is the 2nd half of U1/U2 -> U0 workaround.
	 * See dwc3_gadget_linksts_change_interrupt() for 1st half.
	 */
	if (dwc->revision < DWC3_REVISION_183A) {
		u32		reg;
		int		i;

		for (i = 0; i < DWC3_ENDPOINTS_NUM; i++) {
			dep = dwc->eps[i];

			if (!(dep->flags & DWC3_EP_ENABLED))
				continue;

			if (!list_empty(&dep->req_queued))
				return;
		}

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg |= dwc->u1u2;
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);

		dwc->u1u2 = 0;
	}
}

static void dwc3_endpoint_interrupt(struct dwc3 *dwc,
		const struct dwc3_event_depevt *event)
{
	struct dwc3_ep		*dep;
	u8			epnum = event->endpoint_number;

	dep = dwc->eps[epnum];

	if (!(dep->flags & DWC3_EP_ENABLED))
		return;

	dev_vdbg(dwc->dev, "%s: %s\n", dep->name,
			dwc3_ep_event_string(event->endpoint_event));

	if (epnum == 0 || epnum == 1) {
		dwc3_ep0_interrupt(dwc, event);
		return;
	}

	switch (event->endpoint_event) {
	case DWC3_DEPEVT_XFERCOMPLETE:
		dep->resource_index = 0;
		dep->dbg_ep_events.xfercomplete++;

		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dev_dbg(dwc->dev, "%s is an Isochronous endpoint\n",
					dep->name);
			return;
		}

		dbg_event(dep->number, "XFRCOMP", 0);
		dwc3_endpoint_transfer_complete(dwc, dep, event, 1);
		break;
	case DWC3_DEPEVT_XFERINPROGRESS:
		dep->dbg_ep_events.xferinprogress++;
		if (!usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dev_dbg(dwc->dev, "%s is not an Isochronous endpoint\n",
					dep->name);
			return;
		}

		dbg_event(dep->number, "XFRPROG", 0);
		dwc3_endpoint_transfer_complete(dwc, dep, event, 0);
		break;
	case DWC3_DEPEVT_XFERNOTREADY:
		dbg_event(dep->number, "XFRNRDY", 0);
		dep->dbg_ep_events.xfernotready++;
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dwc3_gadget_start_isoc(dwc, dep, event);
		} else {
			int ret;

			dev_vdbg(dwc->dev, "%s: reason %s\n",
					dep->name, event->status &
					DEPEVT_STATUS_TRANSFER_ACTIVE
					? "Transfer Active"
					: "Transfer Not Active");

			ret = __dwc3_gadget_kick_transfer(dep, 0, 1);
			if (!ret || ret == -EBUSY)
				return;
			else
				dbg_event(dep->number, "QUEUE", ret);

			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}

		break;
	case DWC3_DEPEVT_STREAMEVT:
		dep->dbg_ep_events.streamevent++;
		if (!usb_endpoint_xfer_bulk(dep->endpoint.desc)) {
			dev_err(dwc->dev, "Stream event for non-Bulk %s\n",
					dep->name);
			return;
		}

		switch (event->status) {
		case DEPEVT_STREAMEVT_FOUND:
			dev_vdbg(dwc->dev, "Stream %d found and started\n",
					event->parameters);

			break;
		case DEPEVT_STREAMEVT_NOTFOUND:
			/* FALLTHROUGH */
		default:
			dev_dbg(dwc->dev, "Couldn't find suitable stream\n");
		}
		break;
	case DWC3_DEPEVT_RXTXFIFOEVT:
		dev_dbg(dwc->dev, "%s FIFO Overrun\n", dep->name);
		dep->dbg_ep_events.rxtxfifoevent++;
		break;
	case DWC3_DEPEVT_EPCMDCMPLT:
		dev_vdbg(dwc->dev, "Endpoint Command Complete\n");
		dep->dbg_ep_events.epcmdcomplete++;
		break;
	}
}

static void dwc3_disconnect_gadget(struct dwc3 *dwc)
{
	if (dwc->gadget_driver && dwc->gadget_driver->disconnect) {
		spin_unlock(&dwc->lock);
		dwc->gadget_driver->disconnect(&dwc->gadget);
		spin_lock(&dwc->lock);
	}
	dwc->gadget.xfer_isr_count = 0;
}

static void dwc3_stop_active_transfer(struct dwc3 *dwc, u32 epnum)
{
	struct dwc3_ep *dep;
	struct dwc3_gadget_ep_cmd_params params;
	u32 cmd;
	int ret;

	dep = dwc->eps[epnum];

	if (!dep->resource_index)
		return;

	/*
	 * NOTICE: We are violating what the Databook says about the
	 * EndTransfer command. Ideally we would _always_ wait for the
	 * EndTransfer Command Completion IRQ, but that's causing too
	 * much trouble synchronizing between us and gadget driver.
	 *
	 * We have discussed this with the IP Provider and it was
	 * suggested to giveback all requests here, but give HW some
	 * extra time to synchronize with the interconnect. We're using
	 * an arbitraty 100us delay for that.
	 *
	 * Note also that a similar handling was tested by Synopsys
	 * (thanks a lot Paul) and nothing bad has come out of it.
	 * In short, what we're doing is:
	 *
	 * - Issue EndTransfer WITH CMDIOC bit set
	 * - Wait 100us
	 */

	cmd = DWC3_DEPCMD_ENDTRANSFER;
	cmd |= DWC3_DEPCMD_HIPRI_FORCERM | DWC3_DEPCMD_CMDIOC;
	cmd |= DWC3_DEPCMD_PARAM(dep->resource_index);
	memset(&params, 0, sizeof(params));
	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);
	WARN_ON_ONCE(ret);
	dep->resource_index = 0;
	dep->flags &= ~DWC3_EP_BUSY;
	udelay(100);
}

static void dwc3_stop_active_transfers(struct dwc3 *dwc)
{
	u32 epnum;

	for (epnum = 2; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		struct dwc3_ep *dep;

		dep = dwc->eps[epnum];
		if (!dep)
			continue;

		if (!(dep->flags & DWC3_EP_ENABLED))
			continue;

		dwc3_remove_requests(dwc, dep);
	}
}

static void dwc3_clear_stall_all_ep(struct dwc3 *dwc)
{
	u32 epnum;

	for (epnum = 1; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		struct dwc3_ep *dep;
		struct dwc3_gadget_ep_cmd_params params;
		int ret;

		dep = dwc->eps[epnum];
		if (!dep)
			continue;

		if (!(dep->flags & DWC3_EP_STALL))
			continue;

		dep->flags &= ~DWC3_EP_STALL;

		memset(&params, 0, sizeof(params));
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
				DWC3_DEPCMD_CLEARSTALL, &params);
		WARN_ON_ONCE(ret);
	}
}

static void dwc3_gadget_disconnect_interrupt(struct dwc3 *dwc)
{
	int			reg;

	pr_info("usb:: %s\n", __func__);

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_INITU1ENA;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	reg &= ~DWC3_DCTL_INITU2ENA;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	dbg_event(0xFF, "DISCONNECT", 0);
	dwc3_disconnect_gadget(dwc);
	dwc->start_config_issued = false;

	dwc->gadget.speed = USB_SPEED_UNKNOWN;
	dwc->setup_packet_pending = false;
	dwc->link_state = dwc3_get_link_state(dwc);
}

static void dwc3_gadget_reset_interrupt(struct dwc3 *dwc)
{
	u32			reg;
	struct dwc3_otg		*dotg = dwc->dotg;

	pr_info("usb:: %s\n", __func__);

	/*
	 * WORKAROUND: DWC3 revisions <1.88a have an issue which
	 * would cause a missing Disconnect Event if there's a
	 * pending Setup Packet in the FIFO.
	 *
	 * There's no suggested workaround on the official Bug
	 * report, which states that "unless the driver/application
	 * is doing any special handling of a disconnect event,
	 * there is no functional issue".
	 *
	 * Unfortunately, it turns out that we _do_ some special
	 * handling of a disconnect event, namely complete all
	 * pending transfers, notify gadget driver of the
	 * disconnection, and so on.
	 *
	 * Our suggested workaround is to follow the Disconnect
	 * Event steps here, instead, based on a setup_packet_pending
	 * flag. Such flag gets set whenever we have a XferNotReady
	 * event on EP0 and gets cleared on XferComplete for the
	 * same endpoint.
	 *
	 * Refers to:
	 *
	 * STAR#9000466709: RTL: Device : Disconnect event not
	 * generated if setup packet pending in FIFO
	 */
	if (dwc->revision < DWC3_REVISION_188A) {
		if (dwc->setup_packet_pending)
			dwc3_gadget_disconnect_interrupt(dwc);
	}

	dbg_event(0xFF, "BUS RST", 0);
	/* after reset -> Default State */
	usb_gadget_set_state(&dwc->gadget, USB_STATE_DEFAULT);

	if (dotg && dotg->otg.phy)
		usb_phy_set_power(dotg->otg.phy, 0);

	if (dwc->gadget.speed != USB_SPEED_UNKNOWN)
		dwc3_disconnect_gadget(dwc);

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_TSTCTRL_MASK;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	dwc->test_mode = false;

	dwc3_stop_active_transfers(dwc);
	dwc3_clear_stall_all_ep(dwc);
	dwc->start_config_issued = false;

	/* Reset device address to zero */
	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg &= ~(DWC3_DCFG_DEVADDR_MASK);
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);
	dwc->link_state = dwc3_get_link_state(dwc);
}

static void dwc3_update_ram_clk_sel(struct dwc3 *dwc, u32 speed)
{
	u32 reg;
	u32 usb30_clock = DWC3_GCTL_CLK_BUS;

	/*
	 * We change the clock only at SS but I dunno why I would want to do
	 * this. Maybe it becomes part of the power saving plan.
	 */

	if (speed != DWC3_DSTS_SUPERSPEED)
		return;

	/*
	 * RAMClkSel is reset to 0 after USB reset, so it must be reprogrammed
	 * each time on Connect Done.
	 */
	if (!usb30_clock)
		return;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg |= DWC3_GCTL_RAMCLKSEL(usb30_clock);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

static void dwc3_gadget_conndone_interrupt(struct dwc3 *dwc)
{
	struct dwc3_ep		*dep;
	int			ret;
	u32			reg;
	u8			speed;

	dev_vdbg(dwc->dev, "%s\n", __func__);

	reg = dwc3_readl(dwc->regs, DWC3_DSTS);
	speed = reg & DWC3_DSTS_CONNECTSPD;
	dwc->speed = speed;

	dwc3_update_ram_clk_sel(dwc, speed);

	switch (speed) {
	case DWC3_DCFG_SUPERSPEED:
		/*
		 * WORKAROUND: DWC3 revisions <1.90a have an issue which
		 * would cause a missing USB3 Reset event.
		 *
		 * In such situations, we should force a USB3 Reset
		 * event by calling our dwc3_gadget_reset_interrupt()
		 * routine.
		 *
		 * Refers to:
		 *
		 * STAR#9000483510: RTL: SS : USB3 reset event may
		 * not be generated always when the link enters poll
		 */
		if (dwc->revision < DWC3_REVISION_190A)
			dwc3_gadget_reset_interrupt(dwc);

		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);
		dwc->gadget.ep0->maxpacket = 512;
		dwc->gadget.speed = USB_SPEED_SUPER;
		pr_info("usb:: %s (SS)\n", __func__);
		break;
	case DWC3_DCFG_HIGHSPEED:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		dwc->gadget.ep0->maxpacket = 64;
		dwc->gadget.speed = USB_SPEED_HIGH;
		pr_info("usb:: %s (HS)\n", __func__);
		break;
	case DWC3_DCFG_FULLSPEED2:
	case DWC3_DCFG_FULLSPEED1:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		dwc->gadget.ep0->maxpacket = 64;
		dwc->gadget.speed = USB_SPEED_FULL;
		pr_info("usb:: %s (FS)\n", __func__);
		break;
	case DWC3_DCFG_LOWSPEED:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(8);
		dwc->gadget.ep0->maxpacket = 8;
		dwc->gadget.speed = USB_SPEED_LOW;
		pr_info("usb:: %s (LS)\n", __func__);
		break;
	}

	/* Enable USB2 LPM Capability */

	if ((dwc->revision > DWC3_REVISION_194A)
			&& (speed != DWC3_DCFG_SUPERSPEED)) {
#ifndef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
		reg = dwc3_readl(dwc->regs, DWC3_DCFG);
		reg |= DWC3_DCFG_LPM_CAP;
		dwc3_writel(dwc->regs, DWC3_DCFG, reg);
#endif

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~(DWC3_DCTL_HIRD_THRES_MASK | DWC3_DCTL_L1_HIBER_EN);

		/*
		 * TODO: This should be configurable. For now using
		 * maximum allowed HIRD threshold value of 0b1100
		 */
		reg |= DWC3_DCTL_HIRD_THRES(12);

		dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	}

	dep = dwc->eps[0];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, true);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return;
	}

	dep = dwc->eps[1];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, true);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return;
	}

	/*
	 * Configure PHY via GUSB3PIPECTLn if required.
	 *
	 * Update GTXFIFOSIZn
	 *
	 * In both cases reset values should be sufficient.
	 */
#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
	/*
	 * Incase of K-Prj we want to Probe whether Host for super speed
	 * We Start the UDC with speed_limit always USB_SPEED_SUPER.
	 * if we connect at USB_SPEED_SUPER then we set that the ss_host_avail = 1
	 * We then change the speed_limit to USB_HIGH_SPEED and schedule
	 * a work item to reconnect at HIGH SPEED.
	 * In factory mode we do not need this logic. we should always connect USB3.0
	 */
#ifndef CONFIG_SEC_FACTORY
	if (dwc->ss_host_avail == -1) {
		if (dwc->gadget.speed == USB_SPEED_SUPER) {
			dwc->ss_host_avail = 1;
			dwc->speed_limit = USB_SPEED_HIGH;
			dwc->reconnect = true;
		} else {
			printk(KERN_ERR"usb: Super speed host not available \n");
			dwc->ss_host_avail = 0;
		}
	}
#endif
#endif
}

static void dwc3_gadget_wakeup_interrupt(struct dwc3 *dwc)
{
	pr_info("usb:: %s\n", __func__);

	/* Only perform resume from L2 or Early Suspend states */
	if (dwc->link_state == DWC3_LINK_STATE_U3) {
		dbg_event(0xFF, "WAKEUP", 0);

		dwc->link_state = dwc3_get_link_state(dwc);
		/*
		 * gadget_driver resume function might require some dwc3-gadget
		 * operations, such as ep_enable. Hence, dwc->lock must be
		 * released.
		 */
		spin_unlock(&dwc->lock);
		dwc->gadget_driver->resume(&dwc->gadget);
		spin_lock(&dwc->lock);
	} else {
		dwc->link_state = dwc3_get_link_state(dwc);
	}
}

static void dwc3_gadget_linksts_change_interrupt(struct dwc3 *dwc,
		unsigned int evtinfo)
{
	enum dwc3_link_state	next = evtinfo & DWC3_LINK_STATE_MASK;
	unsigned int		pwropt;

	/*
	 * WORKAROUND: DWC3 < 2.50a have an issue when configured without
	 * Hibernation mode enabled which would show up when device detects
	 * host-initiated U3 exit.
	 *
	 * In that case, device will generate a Link State Change Interrupt
	 * from U3 to RESUME which is only necessary if Hibernation is
	 * configured in.
	 *
	 * There are no functional changes due to such spurious event and we
	 * just need to ignore it.
	 *
	 * Refers to:
	 *
	 * STAR#9000570034 RTL: SS Resume event generated in non-Hibernation
	 * operational mode
	 */
	pwropt = DWC3_GHWPARAMS1_EN_PWROPT(dwc->hwparams.hwparams1);
	if ((dwc->revision < DWC3_REVISION_250A) &&
			(pwropt != DWC3_GHWPARAMS1_EN_PWROPT_HIB)) {
		if ((dwc->link_state == DWC3_LINK_STATE_U3) &&
				(next == DWC3_LINK_STATE_RESUME)) {
			dev_vdbg(dwc->dev, "ignoring transition U3 -> Resume\n");
			return;
		}
	}

	/*
	 * WORKAROUND: DWC3 Revisions <1.83a have an issue which, depending
	 * on the link partner, the USB session might do multiple entry/exit
	 * of low power states before a transfer takes place.
	 *
	 * Due to this problem, we might experience lower throughput. The
	 * suggested workaround is to disable DCTL[12:9] bits if we're
	 * transitioning from U1/U2 to U0 and enable those bits again
	 * after a transfer completes and there are no pending transfers
	 * on any of the enabled endpoints.
	 *
	 * This is the first half of that workaround.
	 *
	 * Refers to:
	 *
	 * STAR#9000446952: RTL: Device SS : if U1/U2 ->U0 takes >128us
	 * core send LGO_Ux entering U0
	 */
	if (dwc->revision < DWC3_REVISION_183A) {
		if (next == DWC3_LINK_STATE_U0) {
			u32	u1u2;
			u32	reg;

			switch (dwc->link_state) {
			case DWC3_LINK_STATE_U1:
			case DWC3_LINK_STATE_U2:
				reg = dwc3_readl(dwc->regs, DWC3_DCTL);
				u1u2 = reg & (DWC3_DCTL_INITU2ENA
						| DWC3_DCTL_ACCEPTU2ENA
						| DWC3_DCTL_INITU1ENA
						| DWC3_DCTL_ACCEPTU1ENA);

				if (!dwc->u1u2)
					dwc->u1u2 = reg & u1u2;

				reg &= ~u1u2;

				dwc3_writel(dwc->regs, DWC3_DCTL, reg);
				break;
			default:
				/* do nothing */
				break;
			}
		}
	}

	dwc->link_state = next;

	dev_vdbg(dwc->dev, "%s link %d\n", __func__, dwc->link_state);
}

static void dwc3_gadget_suspend_interrupt(struct dwc3 *dwc,
			unsigned int evtinfo)
{
	enum dwc3_link_state    next = evtinfo & DWC3_LINK_STATE_MASK;

	if (next == DWC3_LINK_STATE_U3) {
		dbg_event(0xFF, "SUSPEND", 0);
		dwc->gadget_driver->suspend(&dwc->gadget);
	}

	dwc->link_state = next;
	dev_vdbg(dwc->dev, "%s link %d\n", __func__, dwc->link_state);
}

#if 0
static void dwc3_dump_reg_info(struct dwc3 *dwc)
{
	dbg_event(0xFF, "REGDUMP", 0);

	dbg_print_reg("GUSB3PIPCTL", dwc3_readl(dwc->regs,
							DWC3_GUSB3PIPECTL(0)));
	dbg_print_reg("GUSB2PHYCONFIG", dwc3_readl(dwc->regs,
							DWC3_GUSB2PHYCFG(0)));
	dbg_print_reg("GCTL", dwc3_readl(dwc->regs, DWC3_GCTL));
	dbg_print_reg("GUCTL", dwc3_readl(dwc->regs, DWC3_GUCTL));
	dbg_print_reg("GDBGLTSSM", dwc3_readl(dwc->regs, DWC3_GDBGLTSSM));
	dbg_print_reg("DCFG", dwc3_readl(dwc->regs, DWC3_DCFG));
	dbg_print_reg("DCTL", dwc3_readl(dwc->regs, DWC3_DCTL));
	dbg_print_reg("DEVTEN", dwc3_readl(dwc->regs, DWC3_DEVTEN));
	dbg_print_reg("DSTS", dwc3_readl(dwc->regs, DWC3_DSTS));
	dbg_print_reg("DALPENA", dwc3_readl(dwc->regs, DWC3_DALEPENA));
	dbg_print_reg("DGCMD", dwc3_readl(dwc->regs, DWC3_DGCMD));

	dbg_print_reg("OCFG", dwc3_readl(dwc->regs, DWC3_OCFG));
	dbg_print_reg("OCTL", dwc3_readl(dwc->regs, DWC3_OCTL));
	dbg_print_reg("OEVT", dwc3_readl(dwc->regs, DWC3_OEVT));
	dbg_print_reg("OSTS", dwc3_readl(dwc->regs, DWC3_OSTS));
}
#endif

static void dwc3_gadget_interrupt(struct dwc3 *dwc,
		const struct dwc3_event_devt *event)
{
	switch (event->type) {
	case DWC3_DEVICE_EVENT_DISCONNECT:
		dwc3_gadget_disconnect_interrupt(dwc);
		dwc->dbg_gadget_events.disconnect++;
		break;
	case DWC3_DEVICE_EVENT_RESET:
		dwc3_gadget_reset_interrupt(dwc);
		dwc->dbg_gadget_events.reset++;
		break;
	case DWC3_DEVICE_EVENT_CONNECT_DONE:
		dwc3_gadget_conndone_interrupt(dwc);
		dwc->dbg_gadget_events.connect++;
		break;
	case DWC3_DEVICE_EVENT_WAKEUP:
		dwc3_gadget_wakeup_interrupt(dwc);
		dwc->dbg_gadget_events.wakeup++;
		break;
	case DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE:
		dwc3_gadget_linksts_change_interrupt(dwc, event->event_info);
		dwc->dbg_gadget_events.link_status_change++;
		break;
	case DWC3_DEVICE_EVENT_SUSPEND:
		if (dwc->revision < DWC3_REVISION_230A) {
			dev_vdbg(dwc->dev, "End of Periodic Frame\n");
			dwc->dbg_gadget_events.eopf++;
		} else {
			dev_vdbg(dwc->dev, "U3/L1-L2 Suspend Event\n");
			dbg_event(0xFF, "SUSPEND", 0);
			dwc->dbg_gadget_events.suspend++;
			dwc3_gadget_suspend_interrupt(dwc, event->event_info);
		}
		break;
	case DWC3_DEVICE_EVENT_SOF:
		dev_vdbg(dwc->dev, "Start of Periodic Frame\n");
		dwc->dbg_gadget_events.sof++;
		break;
	case DWC3_DEVICE_EVENT_ERRATIC_ERROR:
		if (!dwc->err_evt_seen) {
			dbg_event(0xFF, "ERROR", 0);
			dev_vdbg(dwc->dev, "Erratic Error\n");
		}
		dwc->dbg_gadget_events.erratic_error++;
		break;
	case DWC3_DEVICE_EVENT_CMD_CMPL:
		dev_vdbg(dwc->dev, "Command Complete\n");
		dwc->dbg_gadget_events.cmdcmplt++;
		break;
	case DWC3_DEVICE_EVENT_OVERFLOW:
		dbg_event(0xFF, "OVERFL", 0);
		dev_vdbg(dwc->dev, "Overflow\n");
		/*
		 * Controllers prior to 2.30a revision has a bug where
		 * Overflow Event may overwrite an unacknowledged event
		 * in the event buffer.  The severity of the issue depends
		 * on the overwritten event type.  Add a warning message
		 * saying that an event is overwritten.
		 *
		 * TODO: In future we may need to see if we can re-enumerate
		 * with host.
		 */
		if (dwc->revision < DWC3_REVISION_230A)
			dev_warn(dwc->dev, "Unacknowledged event overwritten\n");
		dwc->dbg_gadget_events.overflow++;
		break;
	case DWC3_DEVICE_EVENT_VENDOR_DEV_TEST_LMP:
		/*
		 * Controllers prior to 2.30a revision has a bug, due to which
		 * a vendor device test LMP event can not be filtered.  But
		 * this event is not handled in the current code.  This is a
		 * special event and 8 bytes of data will follow the event.
		 * Handling this event is tricky when event buffer is almost
		 * full. Moreover this event will not occur in normal scenario
		 * and can only happen with special hosts in testing scenarios.
		 * Add a warning message to indicate that this event is received
		 * which means that event buffer might have corrupted.
		 */
		dbg_event(0xFF, "TSTLMP", 0);
		if (dwc->revision < DWC3_REVISION_230A)
			dev_warn(dwc->dev, "Vendor Device Test LMP Received\n");
		dwc->dbg_gadget_events.vendor_dev_test_lmp++;
		break;
	default:
		pr_info("usb:: [%s] UNKNOWN IRQ %d\n", __func__, event->type);
		dwc->dbg_gadget_events.unknown_event++;
	}

	dwc->err_evt_seen = (event->type == DWC3_DEVICE_EVENT_ERRATIC_ERROR);
}

static void dwc3_process_event_entry(struct dwc3 *dwc,
		const union dwc3_event *event)
{
	/* Endpoint IRQ, handle it and return early */
	if (event->type.is_devspec == 0) {
		/* depevt */
		return dwc3_endpoint_interrupt(dwc, &event->depevt);
	}

	switch (event->type.type) {
	case DWC3_EVENT_TYPE_DEV:
		dwc3_gadget_interrupt(dwc, &event->devt);
		break;
	/* REVISIT what to do with Carkit and I2C events ? */
	default:
		dev_err(dwc->dev, "UNKNOWN IRQ type %d\n", event->raw);
	}
}

static irqreturn_t dwc3_thread_interrupt(int irq, void *_dwc)
{
	struct dwc3 *dwc = _dwc;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	int i;
	unsigned temp_cnt = 0, temp_time;
	ktime_t start_time;

	start_time = ktime_get();

	spin_lock_irqsave(&dwc->lock, flags);

	for (i = 0; i < dwc->num_event_buffers; i++) {
		struct dwc3_event_buffer *evt;
		int			left;

		evt = dwc->ev_buffs[i];
		left = evt->count;
		temp_cnt = temp_cnt + evt->count;

		if (!(evt->flags & DWC3_EVENT_PENDING))
			continue;

		while (left > 0) {
			union dwc3_event event;

			event.raw = *(u32 *) (evt->buf + evt->lpos);

			dwc3_process_event_entry(dwc, &event);

			if (dwc->err_evt_seen) {
				/*
				 * if erratic error, skip remaining events
				 * while controller undergoes reset
				 */
				evt->lpos = (evt->lpos + left) %
						DWC3_EVENT_BUFFERS_SIZE;
				dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(i),
						left);
				if (dwc3_notify_event(dwc,
						DWC3_CONTROLLER_ERROR_EVENT))
					dwc->err_evt_seen = 0;
				break;
			}

			/*
			 * FIXME we wrap around correctly to the next entry as
			 * almost all entries are 4 bytes in size. There is one
			 * entry which has 12 bytes which is a regular entry
			 * followed by 8 bytes data. ATM I don't know how
			 * things are organized if we get next to the a
			 * boundary so I worry about that once we try to handle
			 * that.
			 */
			evt->lpos = (evt->lpos + 4) % DWC3_EVENT_BUFFERS_SIZE;
			left -= 4;

			dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(i), 4);
		}
#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
		/* Schedule the reconnect work event */
		if (dwc->reconnect) {
			dwc->reconnect = false;
			WORK_SCHEDULE(dwc);
		}
#endif
		evt->count = 0;
		evt->flags &= ~DWC3_EVENT_PENDING;
		ret = IRQ_HANDLED;

		if (dwc->err_evt_seen)
			break;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);

	temp_time = ktime_to_us(ktime_sub(ktime_get(), start_time));
	temp_cnt = temp_cnt / 4;
	dwc->bh_handled_evt_cnt[dwc->bh_dbg_index] = temp_cnt;
	dwc->bh_completion_time[dwc->bh_dbg_index] = temp_time;
	dwc->bh_dbg_index = (dwc->bh_dbg_index + 1) % 10;

	pm_runtime_put(dwc->dev);
	return ret;
}

static irqreturn_t dwc3_process_event_buf(struct dwc3 *dwc, u32 buf)
{
	struct dwc3_event_buffer *evt;
	u32 count;

	evt = dwc->ev_buffs[buf];

	count = dwc3_readl(dwc->regs, DWC3_GEVNTCOUNT(buf));
	count &= DWC3_GEVNTCOUNT_MASK;
	if (!count)
		return IRQ_NONE;

	evt->count = count;
	evt->flags |= DWC3_EVENT_PENDING;

	return IRQ_WAKE_THREAD;
}

static void dwc3_interrupt_bh(unsigned long param)
{
	struct dwc3 *dwc = (struct dwc3 *) param;

	pm_runtime_get(dwc->dev);
	dwc3_thread_interrupt(dwc->irq, dwc);
	enable_irq(dwc->irq);
}

static irqreturn_t dwc3_interrupt(int irq, void *_dwc)
{
	struct dwc3			*dwc = _dwc;
	int				i;
	irqreturn_t			ret = IRQ_NONE;

	spin_lock(&dwc->lock);

	dwc->irq_cnt++;

	if (dwc->err_evt_seen) {
		/* controller reset is still pending */
		spin_unlock(&dwc->lock);
		return IRQ_HANDLED;
	}

	for (i = 0; i < dwc->num_event_buffers; i++) {
		irqreturn_t status;

		status = dwc3_process_event_buf(dwc, i);
		if (status == IRQ_WAKE_THREAD)
			ret = status;
	}

	spin_unlock(&dwc->lock);

	if (ret == IRQ_WAKE_THREAD) {
		disable_irq_nosync(irq);
		tasklet_schedule(&dwc->bh);
	}
	return IRQ_HANDLED;
}

/**
 * dwc3_gadget_init - Initializes gadget related registers
 * @dwc: pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
int dwc3_gadget_init(struct dwc3 *dwc)
{
#ifndef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	u32					reg;
#endif
	int					ret;

	dwc->ctrl_req = dma_alloc_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			&dwc->ctrl_req_addr, GFP_KERNEL);
	if (!dwc->ctrl_req) {
		dev_err(dwc->dev, "failed to allocate ctrl request\n");
		ret = -ENOMEM;
		goto err0;
	}

	dwc->ep0_trb = dma_alloc_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			&dwc->ep0_trb_addr, GFP_KERNEL);
	if (!dwc->ep0_trb) {
		dev_err(dwc->dev, "failed to allocate ep0 trb\n");
		ret = -ENOMEM;
		goto err1;
	}

	dwc->setup_buf = kzalloc(DWC3_EP0_BOUNCE_SIZE, GFP_KERNEL);
	if (!dwc->setup_buf) {
		dev_err(dwc->dev, "failed to allocate setup buffer\n");
		ret = -ENOMEM;
		goto err2;
	}

	dwc->ep0_bounce = dma_alloc_coherent(dwc->dev,
			DWC3_EP0_BOUNCE_SIZE, &dwc->ep0_bounce_addr,
			GFP_KERNEL);
	if (!dwc->ep0_bounce) {
		dev_err(dwc->dev, "failed to allocate ep0 bounce buffer\n");
		ret = -ENOMEM;
		goto err3;
	}

	dwc->bh.func = dwc3_interrupt_bh;
	dwc->bh.data = (unsigned long)dwc;

	dwc->gadget.ops			= &dwc3_gadget_ops;
	if (dwc->maximum_speed == USB_SPEED_SUPER)
		dwc->gadget.max_speed		= USB_SPEED_SUPER;
	else
		dwc->gadget.max_speed		= USB_SPEED_HIGH;

	dwc->gadget.speed		= USB_SPEED_UNKNOWN;
	dwc->gadget.sg_supported	= true;
	dwc->gadget.name		= "dwc3-gadget";

	/*
	 * REVISIT: Here we should clear all pending IRQs to be
	 * sure we're starting from a well known location.
	 */

	ret = dwc3_gadget_init_endpoints(dwc);
	if (ret)
		goto err4;

#ifndef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg |= DWC3_DCFG_LPM_CAP;
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);
#endif

	ret = usb_add_gadget_udc(dwc->dev, &dwc->gadget);
	if (ret) {
		dev_err(dwc->dev, "failed to register udc\n");
		goto err5;
	}

	if (dwc->dotg) {
		/* dwc3 otg driver is active (DRD mode + SRPSupport=1) */
		ret = otg_set_peripheral(&dwc->dotg->otg, &dwc->gadget);
		if (ret) {
			dev_err(dwc->dev, "failed to set peripheral to otg\n");
			goto err5;
		}
	} else {
		pm_runtime_no_callbacks(&dwc->gadget.dev);
		pm_runtime_set_active(&dwc->gadget.dev);
		pm_runtime_enable(&dwc->gadget.dev);
		pm_runtime_get(&dwc->gadget.dev);
	}

	return 0;

err5:
	dwc3_gadget_free_endpoints(dwc);

err4:
	dma_free_coherent(dwc->dev, DWC3_EP0_BOUNCE_SIZE,
			dwc->ep0_bounce, dwc->ep0_bounce_addr);

err3:
	kfree(dwc->setup_buf);

err2:
	dma_free_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			dwc->ep0_trb, dwc->ep0_trb_addr);

err1:
	dma_free_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			dwc->ctrl_req, dwc->ctrl_req_addr);

err0:
	return ret;
}

/* -------------------------------------------------------------------------- */

void dwc3_gadget_exit(struct dwc3 *dwc)
{
	if (dwc->dotg) {
		pm_runtime_put(&dwc->gadget.dev);
		pm_runtime_disable(&dwc->gadget.dev);
	}

	usb_del_gadget_udc(&dwc->gadget);

	dwc3_gadget_free_endpoints(dwc);

	dma_free_coherent(dwc->dev, DWC3_EP0_BOUNCE_SIZE,
			dwc->ep0_bounce, dwc->ep0_bounce_addr);

	kfree(dwc->setup_buf);

	dma_free_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			dwc->ep0_trb, dwc->ep0_trb_addr);

	dma_free_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			dwc->ctrl_req, dwc->ctrl_req_addr);
}

int dwc3_gadget_prepare(struct dwc3 *dwc)
{
	if (dwc->pullups_connected)
		dwc3_gadget_disable_irq(dwc);

	return 0;
}

void dwc3_gadget_complete(struct dwc3 *dwc)
{
	if (dwc->pullups_connected) {
		dwc3_gadget_enable_irq(dwc);
		dwc3_gadget_run_stop(dwc, true);
	}
}

int dwc3_gadget_suspend(struct dwc3 *dwc)
{
	__dwc3_gadget_ep_disable(dwc->eps[0]);
	__dwc3_gadget_ep_disable(dwc->eps[1]);

	dwc->dcfg = dwc3_readl(dwc->regs, DWC3_DCFG);

	return 0;
}

int dwc3_gadget_resume(struct dwc3 *dwc)
{
	struct dwc3_ep		*dep;
	int			ret;

	/* Start with SuperSpeed Default */
	dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);

	dep = dwc->eps[0];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret)
		goto err0;

	dep = dwc->eps[1];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret)
		goto err1;

	/* begin to receive SETUP packets */
	dwc->ep0state = EP0_SETUP_PHASE;
	dwc3_ep0_out_start(dwc);

	dwc3_writel(dwc->regs, DWC3_DCFG, dwc->dcfg);

	return 0;

err1:
	__dwc3_gadget_ep_disable(dwc->eps[0]);

err0:
	return ret;
}

#ifdef CONFIG_USB_ANDROID_SAMSUNG_USB30_MENU
int sec_set_speedlimit(struct usb_gadget *gadget,
			enum usb_device_speed speed)
{
	struct dwc3 *dwc;
	unsigned long flags;

	if (!gadget)
		return -1;

	if (speed > gadget->max_speed || speed < USB_SPEED_FULL)
		return -1;

	dwc = container_of(gadget, struct dwc3, gadget);

	spin_lock_irqsave(&dwc->lock, flags);
	dwc->speed_limit = speed;
	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(sec_set_speedlimit);

int sec_get_ss_host_available(struct usb_gadget *gadget)
{
	struct dwc3 *dwc;
	int      ss_host_avail;
	unsigned long flags;

	if (!gadget)
		return -1;
	dwc = container_of(gadget, struct dwc3, gadget);
	spin_lock_irqsave(&dwc->lock, flags);
	ss_host_avail = dwc->ss_host_avail;
	spin_unlock_irqrestore(&dwc->lock, flags);
	dev_dbg(dwc->dev,"Superspeed Host avail(%d) \n",ss_host_avail);
	return ss_host_avail;
}
EXPORT_SYMBOL_GPL(sec_get_ss_host_available);
#endif
