/*
 * Copyright (c) 2013-2014, Linux Foundation. All rights reserved.
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
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/freezer.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/timer.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/q6lsm.h>
#include <sound/lsm_params.h>
#include <sound/pcm_params.h>
#include "msm-pcm-routing-v2.h"

#define CAPTURE_MIN_NUM_PERIODS     2
#define CAPTURE_MAX_NUM_PERIODS     8
#define CAPTURE_MAX_PERIOD_SIZE     4096
#define CAPTURE_MIN_PERIOD_SIZE     320

static struct snd_pcm_hardware msm_pcm_hardware_capture = {
	.info =                 (SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_BLOCK_TRANSFER |
				SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats =              SNDRV_PCM_FMTBIT_S16_LE,
	.rates =                SNDRV_PCM_RATE_16000,
	.rate_min =             16000,
	.rate_max =             16000,
	.channels_min =         1,
	.channels_max =         1,
	.buffer_bytes_max =     CAPTURE_MAX_NUM_PERIODS *
				CAPTURE_MAX_PERIOD_SIZE,
	.period_bytes_min =	CAPTURE_MIN_PERIOD_SIZE,
	.period_bytes_max =     CAPTURE_MAX_PERIOD_SIZE,
	.periods_min =          CAPTURE_MIN_NUM_PERIODS,
	.periods_max =          CAPTURE_MAX_NUM_PERIODS,
	.fifo_size =            0,
};

/* Conventional and unconventional sample rate supported */
static unsigned int supported_sample_rates[] = {
	16000,
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(supported_sample_rates),
	.list = supported_sample_rates,
	.mask = 0,
};

struct lsm_priv {
	struct snd_pcm_substream *substream;
	struct lsm_client *lsm_client;

	struct snd_lsm_event_status *event_status;
	spinlock_t event_lock;
	wait_queue_head_t event_wait;
	unsigned long event_avail;
	atomic_t event_wait_stop;
};

static void lsm_event_handler(uint32_t opcode, uint32_t token,
			      void *payload, void *priv)
{
	unsigned long flags;
	struct lsm_priv *prtd = priv;
	struct snd_pcm_substream *substream = prtd->substream;
	uint16_t status = 0;
	uint16_t payload_size = 0;
	uint16_t index = 0;

	pr_debug("%s: Opcode 0x%x\n", __func__, opcode);
	switch (opcode) {
	case LSM_SESSION_EVENT_DETECTION_STATUS:
		status = (uint16_t)((uint8_t *)payload)[0];
		payload_size = (uint16_t)((uint8_t *)payload)[2];
		index = 4;
		pr_debug("%s: event detect status = %d payload size = %d\n",
			 __func__, status , payload_size);
	break;
	case LSM_SESSION_EVENT_DETECTION_STATUS_V2:
		status = (uint16_t)((uint8_t *)payload)[0];
		payload_size = (uint16_t)((uint8_t *)payload)[1];
		index = 2;
		pr_debug("%s: event detect status = %d payload size = %d\n",
			 __func__, status , payload_size);
		break;
	default:
		pr_debug("%s: Unsupported Event opcode 0x%x\n", __func__,
			 opcode);
		break;
	}
	if (opcode == LSM_SESSION_EVENT_DETECTION_STATUS ||
		opcode == LSM_SESSION_EVENT_DETECTION_STATUS_V2) {
		spin_lock_irqsave(&prtd->event_lock, flags);
		prtd->event_status = krealloc(prtd->event_status,
					sizeof(struct snd_lsm_event_status) +
					payload_size, GFP_ATOMIC);
		prtd->event_status->status = status;
		prtd->event_status->payload_size = payload_size;
		if (likely(prtd->event_status)) {
			memcpy(prtd->event_status->payload,
			       &((uint8_t *)payload)[index],
			       payload_size);
			prtd->event_avail = 1;
			spin_unlock_irqrestore(&prtd->event_lock, flags);
			wake_up(&prtd->event_wait);
		} else {
			spin_unlock_irqrestore(&prtd->event_lock, flags);
			pr_err("%s: Couldn't allocate %d bytes of memory\n",
			       __func__, payload_size);
		}
		if (substream->timer_running)
			snd_timer_interrupt(substream->timer, 1);
	}
}

static int msm_lsm_get_conf_levels(struct lsm_client *client,
				   u8 *conf_levels_ptr)
{
	int rc = 0;

	if (client->num_confidence_levels == 0) {
		pr_debug("%s: no confidence levels provided\n",
			__func__);
		client->confidence_levels = NULL;
		goto done;
	}

	client->confidence_levels =
		kzalloc((sizeof(uint8_t) * client->num_confidence_levels),
			 GFP_KERNEL);
	if (!client->confidence_levels) {
		pr_err("%s: No memory for confidence\n"
			"levels num of level from user = %d\n",
			__func__, client->num_confidence_levels);
			rc = -ENOMEM;
			goto done;
	}

	if (copy_from_user(client->confidence_levels,
			   conf_levels_ptr,
			   client->num_confidence_levels)) {
		pr_err("%s: copy from user failed, size = %d\n",
		       __func__, client->num_confidence_levels);
		rc = -EFAULT;
		goto copy_err;
	}

	return rc;

copy_err:
	kfree(client->confidence_levels);
	client->confidence_levels = NULL;
done:
	return rc;

}

static int msm_lsm_ioctl_shared(struct snd_pcm_substream *substream,
			 unsigned int cmd, void *arg)
{
	unsigned long flags;
	int ret;
	struct snd_lsm_sound_model snd_model;
	struct snd_lsm_sound_model_v2 snd_model_v2;
	struct snd_lsm_session_data session_data;
	int rc = 0;
	int xchg = 0;
	u32 size = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lsm_priv *prtd = runtime->private_data;
	struct snd_lsm_event_status *user = arg;
	struct snd_lsm_detection_params det_params;
	uint8_t *confidence_level = NULL;

	pr_debug("%s: enter cmd %x\n", __func__, cmd);
	switch (cmd) {
	case SNDRV_LSM_SET_SESSION_DATA:
		pr_debug("%s: set Session data\n", __func__);
		memcpy(&session_data, arg,
		       sizeof(struct snd_lsm_session_data));
		if (prtd) {
			if (session_data.app_id <= LSM_VOICE_WAKEUP_APP_ID_V2
			    && session_data.app_id > 0)
				prtd->lsm_client->app_id = session_data.app_id;
			else {
				pr_err("%s:Invalid App id for Listen client\n",
				       __func__);
				rc = -EINVAL;
			}
		} else {
			pr_err("%s: LSM Priv data is NULL\n", __func__);
			rc = -EINVAL;
		}
		break;
	case SNDRV_LSM_REG_SND_MODEL_V2:
		pr_debug("%s: Registering sound model V2\n", __func__);
		memcpy(&snd_model_v2, arg,
		       sizeof(struct snd_lsm_sound_model_v2));
		if (snd_model_v2.num_confidence_levels >
		    MAX_NUM_CONFIDENCE) {
			pr_err("%s: Invalid conf_levels = %d, maximum allowed = %d\n",
				__func__, snd_model_v2.num_confidence_levels,
				MAX_NUM_CONFIDENCE);
			rc = -EINVAL;
			break;
		}

		prtd->lsm_client->snd_model_ver_inuse = SND_MODEL_IN_USE_V2;
		rc = q6lsm_snd_model_buf_alloc(prtd->lsm_client,
					       snd_model_v2.data_size);
		if (rc) {
			pr_err("%s: q6lsm buffer alloc failed V2, size %d\n",
			       __func__, snd_model_v2.data_size);
			break;
		}
		if (copy_from_user(prtd->lsm_client->sound_model.data,
			   snd_model_v2.data, snd_model_v2.data_size)) {
			pr_err("%s: copy from user data failed\n"
			       "data %p size %d\n", __func__,
			       snd_model_v2.data, snd_model_v2.data_size);
			q6lsm_snd_model_buf_free(prtd->lsm_client);
			rc = -EFAULT;
			break;
		}

		pr_debug("SND Model Magic no byte[0] %x,\n"
			 "byte[1] %x, byte[2] %x byte[3] %x\n",
			 snd_model_v2.data[0], snd_model_v2.data[1],
			 snd_model_v2.data[2], snd_model_v2.data[3]);

		prtd->lsm_client->num_confidence_levels =
			snd_model_v2.num_confidence_levels;

		rc = msm_lsm_get_conf_levels(prtd->lsm_client,
				snd_model_v2.confidence_level);
		if (rc) {
			pr_err("%s: get_conf_levels failed, err = %d\n",
				__func__, rc);
			break;
		}

		rc = q6lsm_register_sound_model(prtd->lsm_client,
					snd_model_v2.detection_mode,
					snd_model_v2.detect_failure);
		if (rc < 0) {
			pr_err("%s: Register snd Model v2 failed =%d\n",
			       __func__, rc);
			kfree(confidence_level);
			q6lsm_snd_model_buf_free(prtd->lsm_client);
		}

		kfree(prtd->lsm_client->confidence_levels);
		prtd->lsm_client->confidence_levels = NULL;
		break;

	case SNDRV_LSM_SET_PARAMS:
		if (!arg) {
			pr_err("%s: %s Invalid argument\n",
				__func__, "SNDRV_LSM_SET_PARAMS");
			return -EINVAL;
		}

		memcpy(&det_params, arg,
			sizeof(det_params));
		if (det_params.num_confidence_levels >
		    MAX_NUM_CONFIDENCE) {
			rc = -EINVAL;
			break;
		}

		prtd->lsm_client->num_confidence_levels =
			det_params.num_confidence_levels;

		rc = msm_lsm_get_conf_levels(prtd->lsm_client,
				det_params.conf_level);
		if (rc) {
			pr_err("%s: Failed to get conf_levels, err = %d\n",
				__func__, rc);
			break;
		}

		rc = q6lsm_set_data(prtd->lsm_client,
			       det_params.detect_mode,
			       det_params.detect_failure);
		if (rc)
			pr_err("%s: Failed to set params, err = %d\n",
				__func__, rc);

		kfree(prtd->lsm_client->confidence_levels);
		prtd->lsm_client->confidence_levels = NULL;

		break;

	case SNDRV_LSM_REG_SND_MODEL:
		pr_debug("%s: Registering sound model\n", __func__);
		memcpy(&snd_model, arg, sizeof(struct snd_lsm_sound_model));
		prtd->lsm_client->snd_model_ver_inuse = SND_MODEL_IN_USE_V1;
		rc = q6lsm_snd_model_buf_alloc(prtd->lsm_client,
					       snd_model.data_size);
		if (rc) {
			pr_err("%s: q6lsm buffer alloc failed, size %d\n",
			       __func__, snd_model.data_size);
			break;
		}

		if (copy_from_user(prtd->lsm_client->sound_model.data,
				   snd_model.data, snd_model.data_size)) {
			pr_err("%s: copy from user data failed data %p size %d\n",
			       __func__, snd_model.data, snd_model.data_size);
			rc = -EFAULT;
			q6lsm_snd_model_buf_free(prtd->lsm_client);
			break;
		}
		rc = q6lsm_set_kw_sensitivity_level(prtd->lsm_client,
						snd_model.min_keyw_confidence,
						snd_model.min_user_confidence);
		if (rc) {
			pr_err("%s: Error in KW sensitivity %x", __func__, rc);
			q6lsm_snd_model_buf_free(prtd->lsm_client);
			break;
		}

		rc = q6lsm_register_sound_model(prtd->lsm_client,
						snd_model.detection_mode,
						snd_model.detect_failure);
		if (rc < 0) {
			pr_err("%s: q6lsm_register_sound_model failed =%d\n",
			       __func__, rc);
			q6lsm_snd_model_buf_free(prtd->lsm_client);
		}

		break;

	case SNDRV_LSM_DEREG_SND_MODEL:
		pr_debug("%s: Deregistering sound model\n", __func__);
		rc = q6lsm_deregister_sound_model(prtd->lsm_client);
		break;

	case SNDRV_LSM_EVENT_STATUS:
		pr_debug("%s: Get event status\n", __func__);
		atomic_set(&prtd->event_wait_stop, 0);
		rc = wait_event_freezable(prtd->event_wait,
				(cmpxchg(&prtd->event_avail, 1, 0) ||
				 (xchg = atomic_cmpxchg(&prtd->event_wait_stop,
							1, 0))));
		pr_debug("%s: wait_event_freezable %d event_wait_stop %d\n",
			 __func__, rc, xchg);
		if (!rc && !xchg) {
			pr_debug("%s: New event available %ld\n", __func__,
				 prtd->event_avail);
			spin_lock_irqsave(&prtd->event_lock, flags);
			if (prtd->event_status) {
				size = sizeof(*(prtd->event_status)) +
				prtd->event_status->payload_size;
				spin_unlock_irqrestore(&prtd->event_lock,
						       flags);
			} else {
				spin_unlock_irqrestore(&prtd->event_lock,
						       flags);
				rc = -EINVAL;
				pr_err("%s: prtd->event_status is NULL\n",
					__func__);
				break;
			}
			if (user->payload_size <
			    prtd->event_status->payload_size) {
				pr_debug("%s: provided %dbytes isn't enough, needs %dbytes\n",
					 __func__, user->payload_size,
					 prtd->event_status->payload_size);
				rc = -ENOMEM;
			} else
				memcpy(user, prtd->event_status, size);
		} else if (xchg) {
			pr_debug("%s: Wait aborted\n", __func__);
			rc = 0;
		}
		break;

	case SNDRV_LSM_ABORT_EVENT:
		pr_debug("%s: Aborting event status wait\n", __func__);
		atomic_set(&prtd->event_wait_stop, 1);
		wake_up(&prtd->event_wait);
		break;

	case SNDRV_LSM_START:
		pr_debug("%s: Starting LSM client session\n", __func__);
		if (!prtd->lsm_client->started) {
			ret = q6lsm_start(prtd->lsm_client, true);
			if (!ret) {
				prtd->lsm_client->started = true;
				pr_debug("%s: LSM client session started\n",
					 __func__);
			}
		}
		break;

	case SNDRV_LSM_STOP:
		pr_debug("%s: Stopping LSM client session\n", __func__);
		if (prtd->lsm_client->started) {
			ret = q6lsm_stop(prtd->lsm_client, true);
			if (!ret)
				pr_debug("%s: LSM client session stopped %d\n",
					 __func__, ret);
			prtd->lsm_client->started = false;
		}
		break;

	default:
		pr_debug("%s: Falling into default snd_lib_ioctl cmd 0x%x\n",
			 __func__, cmd);
		rc = snd_pcm_lib_ioctl(substream, cmd, arg);
		break;
	}

	if (!rc)
		pr_debug("%s: leave (%d)\n", __func__, rc);
	else
		pr_err("%s: cmd 0x%x failed %d\n", __func__, cmd, rc);

	return rc;
}

static int msm_lsm_ioctl(struct snd_pcm_substream *substream,
			 unsigned int cmd, void *arg)
{
	int err = 0;
	u32 size = 0;
	struct snd_lsm_session_data session_data;

	if (!substream) {
		pr_err("%s: Invalid params\n", __func__);
		return -EINVAL;
	}
	switch (cmd) {
	case SNDRV_LSM_SET_SESSION_DATA:
		pr_debug("%s: SNDRV_LSM_SET_SESSION_DATA\n", __func__);
		if (copy_from_user(&session_data, (void *)arg,
				   sizeof(struct snd_lsm_session_data))) {
			err = -EFAULT;
			pr_err("%s: copy from user failed, size %d\n",
			       __func__, sizeof(struct snd_lsm_session_data));
			break;
		}
		if (!err)
			err = msm_lsm_ioctl_shared(substream,
						   cmd, &session_data);
		if (err)
			pr_err("%s REG_SND_MODEL failed err %d\n",
			__func__, err);
		break;
	case SNDRV_LSM_REG_SND_MODEL_V2: {
		struct snd_lsm_sound_model_v2 snd_model_v2;
		if (!arg) {
			pr_err("%s: Invalid params snd_model\n", __func__);
			return -EINVAL;
		}
		if (copy_from_user(&snd_model_v2, arg, sizeof(snd_model_v2))) {
			err = -EFAULT;
			pr_err("%s: copy from user failed, size %zd\n",
			__func__, sizeof(struct snd_lsm_sound_model_v2));
		}
		if (!err)
			err = msm_lsm_ioctl_shared(substream, cmd,
						   &snd_model_v2);
		if (err)
			pr_err("%s REG_SND_MODEL failed err %d\n",
			__func__, err);
		return err;
		}
		break;
	case SNDRV_LSM_REG_SND_MODEL: {
		struct snd_lsm_sound_model snd_model;
		pr_debug("%s: SNDRV_LSM_REG_SND_MODEL\n", __func__);
		if (!arg) {
			pr_err("%s: Invalid params snd_model\n", __func__);
			return -EINVAL;
		}
		if (copy_from_user(&snd_model, arg, sizeof(snd_model))) {
			err = -EFAULT;
			pr_err("%s: copy from user failed, size %zd\n",
			__func__, sizeof(struct snd_lsm_sound_model));
		}
		if (!err)
			err = msm_lsm_ioctl_shared(substream, cmd, &snd_model);
		if (err)
			pr_err("%s REG_SND_MODEL failed err %d\n",
			__func__, err);
		return err;
	}
	case SNDRV_LSM_SET_PARAMS: {
		struct snd_lsm_detection_params det_params;

		pr_debug("%s: SNDRV_LSM_SET_PARAMS\n", __func__);
		if (!arg) {
			pr_err("%s: %s, Invalid params\n",
				__func__, "SNDRV_LSM_SET_PARAMS");
			return -EINVAL;
		}

		if (copy_from_user(&det_params, arg,
				   sizeof(det_params))) {
			pr_err("%s: %s: copy_from_user failed, size %zd\n",
				__func__, "SNDRV_LSM_SET_PARAMS",
				sizeof(det_params));
			err = -EFAULT;
		}

		if (!err)
			err = msm_lsm_ioctl_shared(substream, cmd,
						   &det_params);
		else
			pr_err("%s: LSM_SET_PARAMS failed, err %d\n",
				__func__, err);
		return err;
	}

	case SNDRV_LSM_EVENT_STATUS: {
		struct snd_lsm_event_status *user = NULL, userarg;
		pr_debug("%s: SNDRV_LSM_EVENT_STATUS\n", __func__);
		if (!arg) {
			pr_err("%s: Invalid params event status\n", __func__);
			return -EINVAL;
		}
		if (copy_from_user(&userarg, arg, sizeof(userarg))) {
			pr_err("%s: err copyuser event_status\n",
			__func__);
			return -EFAULT;
		}
		size = sizeof(struct snd_lsm_event_status) +
		userarg.payload_size;
		user = kmalloc(size, GFP_KERNEL);
		if (!user) {
			pr_err("%s: Allocation failed event status size %d\n",
			__func__, size);
			return -EFAULT;
		} else {
			user->payload_size = userarg.payload_size;
			err = msm_lsm_ioctl_shared(substream, cmd, user);
		}
		/* Update size with actual payload size */
		size = sizeof(*user) + user->payload_size;
		if (!err && !access_ok(VERIFY_WRITE, arg, size)) {
			pr_err("%s: write verify failed size %d\n",
			__func__, size);
			err = -EFAULT;
		}
		if (!err && (copy_to_user(arg, user, size))) {
			pr_err("%s: failed to copy payload %d",
			__func__, size);
			err = -EFAULT;
		}
		kfree(user);
		if (err)
			pr_err("%s: lsmevent failed %d", __func__, err);
		return err;
	}
	default:
		err = msm_lsm_ioctl_shared(substream, cmd, arg);
	break;
	}
	return err;
}

static int msm_lsm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lsm_priv *prtd;
	int ret = 0;

	pr_debug("%s\n", __func__);
	prtd = kzalloc(sizeof(struct lsm_priv), GFP_KERNEL);
	if (!prtd) {
		pr_err("%s: Failed to allocate memory for lsm_priv\n",
		       __func__);
		return -ENOMEM;
	}
	spin_lock_init(&prtd->event_lock);
	init_waitqueue_head(&prtd->event_wait);
	prtd->substream = substream;
	runtime->private_data = prtd;
	runtime->hw = msm_pcm_hardware_capture;

	ret = snd_pcm_hw_constraint_list(runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&constraints_sample_rates);
	if (ret < 0)
		pr_info("%s: snd_pcm_hw_constraint_list failed ret %d\n",
			 __func__, ret);
	/* Ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
			    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		pr_info("%s: snd_pcm_hw_constraint_integer failed ret %d\n",
			__func__, ret);

	ret = snd_pcm_hw_constraint_minmax(runtime,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
		CAPTURE_MIN_NUM_PERIODS * CAPTURE_MIN_PERIOD_SIZE,
		CAPTURE_MAX_NUM_PERIODS * CAPTURE_MAX_PERIOD_SIZE);
	if (ret < 0)
		pr_info("%s: constraint for buffer bytes min max ret = %d\n",
			__func__, ret);
	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret < 0) {
		pr_info("%s: constraint for period bytes step ret = %d\n",
			__func__, ret);
	}
	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret < 0)
		pr_info("%s: constraint for buffer bytes step ret = %d\n",
			__func__, ret);
	prtd->lsm_client = q6lsm_client_alloc(
				(lsm_app_cb)lsm_event_handler, prtd);
	if (!prtd->lsm_client) {
		pr_err("%s: Could not allocate memory\n", __func__);
		kfree(prtd);
		runtime->private_data = NULL;
		return -ENOMEM;
	}
	return 0;
}

static int msm_lsm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lsm_priv *prtd = runtime->private_data;
	int ret = 0;

	if (!prtd->lsm_client) {
		pr_err("%s: LSM client data ptr is NULL\n", __func__);
		return -EINVAL;
	}
	ret = q6lsm_open(prtd->lsm_client, prtd->lsm_client->app_id);
	if (ret < 0) {
		pr_err("%s: lsm open failed, %d\n", __func__, ret);
		q6lsm_client_free(prtd->lsm_client);
		kfree(prtd);
		return ret;
	}
	pr_debug("%s: Session ID %d\n", __func__,
		 prtd->lsm_client->session);
	prtd->lsm_client->started = false;
	runtime->private_data = prtd;
	return 0;
}

static int msm_lsm_close(struct snd_pcm_substream *substream)
{
	unsigned long flags;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lsm_priv *prtd = runtime->private_data;
	int ret = 0;

	pr_debug("%s\n", __func__);
	if (prtd->lsm_client->started) {
		ret = q6lsm_stop(prtd->lsm_client, true);
		if (ret)
			pr_err("%s: session stop failed, err = %d\n",
				__func__, ret);
		else
			pr_debug("%s: LSM client session stopped %d\n",
				 __func__, ret);

		/*
		 * Go Ahead and try de-register sound model,
		 * even if stop failed
		 */
		prtd->lsm_client->started = false;

		ret = q6lsm_deregister_sound_model(prtd->lsm_client);
		if (ret)
			pr_err("%s: dereg_snd_model failed, err = %d\n",
				__func__, ret);
		else
			pr_debug("%s: dereg_snd_model succesful\n",
				 __func__);
	}

	q6lsm_close(prtd->lsm_client);
	q6lsm_client_free(prtd->lsm_client);

	spin_lock_irqsave(&prtd->event_lock, flags);
	kfree(prtd->event_status);
	prtd->event_status = NULL;
	spin_unlock_irqrestore(&prtd->event_lock, flags);
	kfree(prtd);

	return 0;
}

static struct snd_pcm_ops msm_lsm_ops = {
	.open           = msm_lsm_open,
	.close          = msm_lsm_close,
	.ioctl          = msm_lsm_ioctl,
	.prepare	= msm_lsm_prepare,
};

static int msm_asoc_lsm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	return 0;
}

static int msm_asoc_lsm_probe(struct snd_soc_platform *platform)
{
	pr_debug("enter %s\n", __func__);

	return 0;
}

static struct snd_soc_platform_driver msm_soc_platform = {
	.ops		= &msm_lsm_ops,
	.pcm_new	= msm_asoc_lsm_new,
	.probe		= msm_asoc_lsm_probe,
};

static int msm_lsm_probe(struct platform_device *pdev)
{
	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", "msm-lsm-client");

	return snd_soc_register_platform(&pdev->dev, &msm_soc_platform);
}

static int msm_lsm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);

	return 0;
}

static const struct of_device_id msm_lsm_client_dt_match[] = {
	{.compatible = "qcom,msm-lsm-client" },
	{ }
};

static struct platform_driver msm_lsm_driver = {
	.driver = {
		.name = "msm-lsm-client",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(msm_lsm_client_dt_match),
	},
	.probe = msm_lsm_probe,
	.remove = msm_lsm_remove,
};

static int __init msm_soc_platform_init(void)
{
	return platform_driver_register(&msm_lsm_driver);
}
module_init(msm_soc_platform_init);

static void __exit msm_soc_platform_exit(void)
{
	platform_driver_unregister(&msm_lsm_driver);
}
module_exit(msm_soc_platform_exit);

MODULE_DESCRIPTION("LSM client platform driver");
MODULE_DEVICE_TABLE(of, msm_lsm_client_dt_match);
MODULE_LICENSE("GPL v2");
