/*
 * Samsung Mobile VE Group.
 *
 * drivers/irda/gpio_ir.c
 *
 * Drivers for samsung IRDA using AP GPIO Control.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *	Jeeon Park <jeeon.park@samsung.com>
 *
 * This program is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/of_gpio.h>

#include <linux/device.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos.h>
#include <asm/arch_timer.h>
#include <linux/cpu.h>

#include "gpio_ir.h"


/****************************************************************/
bool *gir_log_enable;
#define GIR_LOG(fmt, ...)										\
	do {														\
		if (likely(gir_log_enable) && *gir_log_enable)				\
			printk(KERN_INFO fmt, ##__VA_ARGS__);				\
	 } while(0)
/****************************************************************/

#ifdef GPIO_IR_CUSTOM_LOCK_EXYNOS
/****************************************************************/
struct gpio_chip *ir_chip;
struct samsung_gpio_chip *ir_ourchip;
unsigned gpio_offset;

extern void samsung_gpio_ir_gpiolib_set(struct gpio_chip *chip,
				unsigned offset, int value);

#define GPIO_IR_SET_VALUE(info, value)
	do {															\
		if(info->bcustom_lock)										\
			samsung_gpio_ir_gpiolib_set(ir_chip, gpio_offset, (value));		\
		else														\
			gpio_set_value(info->gpio, (value));						\
	 } while(0)
	 
#define GIR_LOCK(flags)												\
	do {															\
		if(info->bcustom_lock)	{									\
			spin_lock_irqsave(&ir_ourchip->lock, flags);					\
		}														\
		else  { 													\
			local_irq_save(flags);									\
			preempt_disable();										\
		}														\
	 } while(0)

#define GIR_UNLOCK(flags)											\
	do {															\
		if(info->bcustom_lock)	{									\
			spin_unlock_irqrestore(&ir_ourchip->lock, flags);				\
		}														\
		else  { 													\
				local_irq_restore(flags);								\
			preempt_enable();										\
		}														\
	 } while(0)
/****************************************************************/
#else

/****************************************************************/
#define GPIO_IR_SET_VALUE(value)		gpio_set_value(info->gpio, (value))
#define GIR_LOCK(flags)										\
	do {														\
		local_irq_save(flags);									\
		preempt_disable();										\
	 } while(0)
	 
#define GIR_UNLOCK(flags)										\
	do {														\
		local_irq_restore(flags);								\
		preempt_enable();										\
	} while(0)
/****************************************************************/
#endif

#ifdef CONFIG_ARM_ARCH_TIMER
static inline unsigned long __timer_delay_get_milli_count(void)
{
	unsigned long long loops;
	unsigned long utime = 1000UL;

	loops = (unsigned long long)utime * (unsigned long long)UDELAY_MULT;
	loops *= arm_delay_ops.ticks_per_jiffy;
	return (unsigned long)(loops >> UDELAY_SHIFT);
}

static unsigned long __timer_delay_get_count(unsigned long nsecs)
{
	unsigned long long loops;
	unsigned long m_part = 0, n_part = 0;
	unsigned long m_count;
	unsigned long delay_cnt = 0;

	if (nsecs > 1000000UL) {
		m_count = __timer_delay_get_milli_count();
		m_part = nsecs /1000000UL;
		n_part = nsecs - (m_part * 1000000UL);

		delay_cnt = m_part *m_count;
		
		loops = (unsigned long long)(n_part) * (unsigned long long)UDELAY_MULT;
		loops *= arm_delay_ops.ticks_per_jiffy;
		delay_cnt += (unsigned long)(loops >> UDELAY_SHIFT) / 1000UL;
		return delay_cnt;
	}
	else	{
		loops = (unsigned long long)(nsecs) * (unsigned long long)UDELAY_MULT;
		loops *= arm_delay_ops.ticks_per_jiffy;
		return (unsigned long)(loops >> UDELAY_SHIFT) / 1000UL;
	}
}


static cycles_t __gpio_ir_timer_delay_from_start(cycles_t start, unsigned long cycles)
{
	cycles_t temp_cycle;

	while (1) {
		temp_cycle = get_cycles();
		if((temp_cycle - start) < cycles)
			cpu_relax();
		else 
			break;
	}
	return temp_cycle;
}

static void __gpio_ir_timer_delay(unsigned long cycles)
{
	cycles_t start = get_cycles();

	while ((get_cycles() - start) < cycles)
		cpu_relax();
}


static unsigned int __timer_custom_ndelay(unsigned long nsecs)
{
	unsigned long long loops = 
		(unsigned long long)nsecs * (unsigned long long)UDELAY_MULT;
	loops *= arm_delay_ops.ticks_per_jiffy;
	//pr_info("[GPIO_IR][%s] nsecs = %lu, lpj_fine = %lu, loops = %llu\n", 
	//	__func__, nsecs, lpj_fine, loops >> UDELAY_SHIFT);
	__gpio_ir_timer_delay((unsigned long)(loops >> UDELAY_SHIFT) / 1000);

	return (unsigned int)(loops >> UDELAY_SHIFT) / 1000;
}
#endif

#ifdef CONFIG_ARM_ARCH_TIMER
#define gpio_ir_new_ndelay(nsecs)	__timer_custom_ndelay(nsecs)
#else
#define gpio_ir_new_ndelay(nsecs)	__loop_custom_ndelay(nsecs)
#endif


/****************************************************************/
static inline void gpio_ir_delay(unsigned long nsecs)
{
#ifdef CONFIG_ARM_ARCH_TIMER
	unsigned long m_part = 0, n_part = 0;
	if (nsecs > 1000000) {
		m_part = nsecs /1000000;
		n_part = nsecs - (m_part * 1000000);
		mdelay(m_part);
		gpio_ir_new_ndelay(n_part);
	}
	else	{
		gpio_ir_new_ndelay(nsecs);
	}
#else
	unsigned long m_part = 0, u_part = 0, n_part = 0;

	m_part = nsecs /1000000;
	u_part = (nsecs - (m_part * 1000000)) / 1000;
	n_part = nsecs - (m_part * 1000000) - (u_part * 1000);
	if (m_part)
		mdelay(m_part);
	if (u_part)
		udelay(u_part);
	if (n_part)
		gpio_ir_new_ndelay(n_part);
#endif


}
/****************************************************************/
#define GIR_TIMER_INIT(TIMER)				\
	hrtimer_init(TIMER, CLOCK_PROCESS_CPUTIME_ID, HRTIMER_MODE_REL)
#define GIR_TIMER_START(TIMER, TIM)	\
	__hrtimer_start_range_ns(TIMER, TIM, 0, HRTIMER_MODE_REL, 1)
/****************************************************************/


/*****************************************************************/
/* sysfs */
/*****************************************************************/
static ssize_t remocon_show	(
	struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t remocon_store(
	struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t remocon_ack(
	struct device *dev, struct device_attribute *attr, char *buf);

#if defined(CONFIG_SEC_FACTORY)
static DEVICE_ATTR(ir_send, 0666, remocon_show, remocon_store);
#else
static DEVICE_ATTR(ir_send, 0664, remocon_show, remocon_store);
#endif
static DEVICE_ATTR(ir_send_result, 0444, remocon_ack, NULL);

static struct attribute *gpio_ir_attributes[] = {
	&dev_attr_ir_send.attr,
	&dev_attr_ir_send_result.attr,
	NULL,
};
static struct attribute_group gpio_ir_attr_group = {
	.attrs = gpio_ir_attributes,
};


/****************************************************************/
/* Define this function in accordance with the state of each BB platform                */
static void irled_power_onoff(int onoff, int power_gpio)
{
	if(onoff)
		gpio_set_value(power_gpio, GPIO_LEVEL_HIGH);
	else
		gpio_set_value(power_gpio, GPIO_LEVEL_LOW);
	return;
}

static void irled_power_init(int power_gpio)
{
	int rc;
	
	rc = gpio_request(power_gpio, "ir_led_en");
	if (rc)
		pr_err("%s: Ir led en error : %d\n", __func__, rc);
	else
		gpio_direction_output(power_gpio, 0);
	
	return;
}
/****************************************************************/


static enum hrtimer_restart gpio_ir_timeout_type1(struct hrtimer *timer)
{
	struct gpio_ir_info_t *info;
	int i;
	unsigned long flags = 0;
	int temp_count;
#ifdef CONFIG_ARM_ARCH_TIMER
	cycles_t start_cycle;
#endif
	info = container_of(timer, struct gpio_ir_info_t, timer);

	if (info->data_pos != 0)
		info->data_pos++;
	else
		pr_info("[GPIO_IR][%s] ir start!\n", __func__);


	GIR_LOCK(flags);
	if (info->bcompleted)
		goto completed;

#ifdef CONFIG_ARM_ARCH_TIMER
	if (info->blow_data_correction) {
		if (info->data_pos != 0) {
			__gpio_ir_timer_delay_from_start(info->low_start_cycle, info->delay_cycles[info->data_pos-1]);
		}
	}
#endif	

	if (info->data_pos < info->count) {
		if (info->ir_data_type == 2)
			temp_count = (info->data[info->data_pos] * 1000) /info->period;
		else
			temp_count = info->data[info->data_pos];

#ifdef CONFIG_ARM_ARCH_TIMER
		GPIO_IR_SET_VALUE(0);
		start_cycle = get_cycles();
		if (info->bhigh_data_correction) {
			info->high_start_cycle = start_cycle;
		}
		for (i = 0; i < temp_count; i++) {
			GPIO_IR_SET_VALUE(1);
			start_cycle = __gpio_ir_timer_delay_from_start(start_cycle, info->up_delay_cycles);
			GPIO_IR_SET_VALUE(0);
			start_cycle = __gpio_ir_timer_delay_from_start(start_cycle, info->down_delay_cycles);
			if (info->bhigh_data_correction && 
					((start_cycle- info->high_start_cycle) >= info->delay_cycles[info->data_pos])) {
				GIR_LOG("[GPIO_IR][%s] High Data Correcton! (%d:%d)\n", 
					__func__, temp_count, i);
				break;
			}
		}
#else
		for (i = 0; i < temp_count; i++) {
			GPIO_IR_SET_VALUE(1);
			gpio_ir_new_ndelay(info->up_delay);
			GPIO_IR_SET_VALUE(0);
			gpio_ir_new_ndelay(info->down_delay);
		}
#endif
	}
	else {
		goto completed;
	}

	info->data_pos++;
	
	if (info->data_pos < info->count) {
#ifdef CONFIG_ARM_ARCH_TIMER
		if (info->blow_data_correction) {
			info->low_start_cycle =  get_cycles();
		}
#endif
		GIR_TIMER_START(timer, info->ktime_delay[info->data_pos]);
		goto no_completed;
	}
	else
		goto completed;

completed:
	if (!info->bcompleted)	{
		info->bcompleted = true;
		GIR_TIMER_START(timer, ktime_set(0, info->end_delay));
	}
	else {
		complete(&info->ir_send_comp);
		pr_info("[GPIO_IR][%s] ir end!\n", __func__);
	}

no_completed:

	GIR_LOG("[GPIO_IR][%s] --\n", __func__);
	GIR_UNLOCK(flags);
	return HRTIMER_NORESTART;
}




static enum hrtimer_restart gpio_ir_timeout_type2(struct hrtimer *timer)
{
	struct gpio_ir_info_t *info;
	int i;
	unsigned long flags = 0;
	int temp_count;
	bool btimer_run;
	cycles_t start_cycle;
	info = container_of(timer, struct gpio_ir_info_t, timer);

	if (info->data_pos != 0)
		info->data_pos++;
	else
		pr_info("[GPIO_IR][%s] ir start!\n", __func__);

	GIR_LOCK(flags);
	if (info->bcompleted)
		goto completed;

#ifdef CONFIG_ARM_ARCH_TIMER
	if (info->blow_data_correction) {
		if (info->data_pos != 0) {
			__gpio_ir_timer_delay_from_start(info->low_start_cycle, info->delay_cycles[info->data_pos-1]);
		}
	}
#endif

	for (; info->data_pos < info->count; info->data_pos++) {
		
		if ((info->data_pos & 0x01) == 0) {
			if (info->ir_data_type == 2)
				temp_count = (info->data[info->data_pos] * 1000) /info->period;
			else
				temp_count = info->data[info->data_pos];

#ifdef CONFIG_ARM_ARCH_TIMER
				GPIO_IR_SET_VALUE(0);
				start_cycle = get_cycles();
				if (info->bhigh_data_correction) {
					info->high_start_cycle = start_cycle;
				}
				for (i = 0; i < temp_count; i++) {
					GPIO_IR_SET_VALUE(1);
					start_cycle = __gpio_ir_timer_delay_from_start(start_cycle, info->up_delay_cycles);
					GPIO_IR_SET_VALUE(0);
					start_cycle = __gpio_ir_timer_delay_from_start(start_cycle, info->down_delay_cycles);
					if (info->bhigh_data_correction && 
							((start_cycle- info->high_start_cycle) >= info->delay_cycles[info->data_pos])) {
						GIR_LOG("[GPIO_IR][%s] High Data Correcton! (%d:%d)\n", 
							__func__, temp_count, i);
						break;
					}
				}
#else
				for (i = 0; i < temp_count; i++) {
					GPIO_IR_SET_VALUE(1);
					gpio_ir_new_ndelay(info->up_delay);
					GPIO_IR_SET_VALUE(0);
					gpio_ir_new_ndelay(info->down_delay);
				}
#endif
		}
		else {
			if (info->ir_data_type == 2) {
				if (info->data[info->data_pos] < 1000)
					btimer_run = false;
				else
					btimer_run = true;
			}
			else {
				if (info->period * info->data[info->data_pos] < 1000000)
					btimer_run = false;
				else
					btimer_run = true;
			}
			if (!btimer_run) {
				gpio_ir_delay(info->low_delay[info->data_pos]);
			}
			else {
#ifdef CONFIG_ARM_ARCH_TIMER
				if (info->blow_data_correction) {
					info->low_start_cycle = get_cycles();
				}
#endif
				GIR_TIMER_START(timer, info->ktime_delay[info->data_pos]);
				goto no_completed;
			}
		}
	}

completed:
	if (!info->bcompleted)	{
		info->bcompleted = true;
		GIR_TIMER_START(timer, ktime_set(0, info->end_delay));
	}
	else {
		complete(&info->ir_send_comp);
		pr_info("[GPIO_IR][%s] ir end!\n", __func__);
	}

no_completed:
	GIR_LOG("[GPIO_IR][%s] --\n", __func__);
	GIR_UNLOCK(flags);
	return HRTIMER_NORESTART;
	
}


static int cpufreq_gpioir_notifier_policy(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_policy *policy = (struct cpufreq_policy *)data;

	GIR_LOG("[GPIO_IR][%s] ++, min(%d) max(%d), cpu(%d)\n", __func__, policy->min, policy->max, policy->cpu);

#ifdef CONFIG_ARM_ARCH_TIMER
	if (val != CPUFREQ_ADJUST)
		return 0;


	if (policy->min < policy->max)
		policy->min = policy->max;
	
#else
	if (val != CPUFREQ_INCOMPATIBLE)
		return 0;

	policy->min = 1728000;		// approriate frequency
	policy->max= 1728000;
#endif
	GIR_LOG("[GPIO_IR][%s] --, min(%d) max(%d), cpu(%d)\n", __func__, policy->min, policy->max, policy->cpu);

	return 0;
}


static struct notifier_block notifier_policy_block = {
	.notifier_call = cpufreq_gpioir_notifier_policy
};

static void gpio_ir_remote_write(struct gpio_ir_info_t *info)
{
	int index;
	int ret = 0;
	int i, j;
	unsigned long flags = 0;
	int temp_count = 0, temp_count2 = 0;
#ifdef CONFIG_ARM_ARCH_TIMER
	cycles_t start_cycle;
#endif
	unsigned long calc_low_delay;

	GIR_LOG("[GPIO_IR][%s] ++\n", __func__);

	info->check_ir_send = 0;			//send result init

	info->period = IR_FREQ_UNIT_GHZ / (ulong)info->freq;
	info->up_delay = info->period /3;
	info->down_delay = info->up_delay * 2;
	GIR_LOG("[GPIO_IR][%s] period (%ld) up delay(%ld) down delay(%ld)\n", 
		__func__, info->period, info->up_delay, info->down_delay);
#ifdef CONFIG_ARM_ARCH_TIMER
	if (info->gpio_high_delay > info->gpio_low_delay) {
		info->up_delay += (info->gpio_high_delay - info->gpio_low_delay);
		info->down_delay -= (info->gpio_high_delay - info->gpio_low_delay);
	} else {
		info->up_delay -= (info->gpio_low_delay - info->gpio_high_delay);
		info->down_delay += (info->gpio_low_delay - info->gpio_high_delay);
	}
#else
	info->up_delay -= info->gpio_low_delay;
	info->down_delay -= info->gpio_high_delay;
#endif
	info->up_delay += info->gpio_custom_delay;
	info->down_delay += info->gpio_custom_delay;

	GIR_LOG("[GPIO_IR][%s] changed up delay(%ld) down delay(%ld)\n", 
		__func__, info->up_delay, info->down_delay);
	GIR_LOG("[GPIO_IR][%s] data count = %d\n", __func__, info->count);

	mutex_lock(&info->mutex_lock);
	irled_power_onoff(1, info->en_gpio);

	if (info->blpm_disable) {
		pm_qos_update_request(&info->pm_qos_req, 1);
	}

	if (info->bir_data_type_auto) {
		for (i = 0; i < info->count; i++) {
			if (info->data[i] <= 0xff)
				temp_count++;
			else
				temp_count2++;
		}
		if (temp_count > temp_count2)
			info->ir_data_type = 1;
		else
			info->ir_data_type = 2;
	}

#ifdef CONFIG_ARM_ARCH_TIMER
	info->up_delay_cycles = __timer_delay_get_count(info->up_delay);
	info->down_delay_cycles = __timer_delay_get_count(info->down_delay);
	GIR_LOG("[GPIO_IR][%s] up delay(%lu), cycles(%lu)\n", 
		__func__, info->up_delay, info->up_delay_cycles);
	GIR_LOG("[GPIO_IR][%s] down delay(%lu), cycles(%lu)\n", 
		__func__, info->down_delay, info->down_delay_cycles);

	if (info->bhigh_data_correction) {
		for (index = 0; index < info->count; index += 2) {
			if (info->ir_data_type == 2)
				info->delay_cycles[index]  =__timer_delay_get_count(info->data[index] * 1000);
			else
				info->delay_cycles[index]  =__timer_delay_get_count( info->period * info->data[index]);
		}
	}
#endif

	for (index = 1; index < info->count; index += 2) {
		if (info->ir_data_type == 2)
			info->low_delay[index] = info->data[index] * 1000;
		else
			info->low_delay[index] = info->period * info->data[index];

		if (info->timer_delay >= 0) {
			calc_low_delay = (info->low_delay[index] < (unsigned long)info->timer_delay ? 
				0 : info->low_delay[index] - info->timer_delay);
		} else {
			calc_low_delay = info->low_delay[index] + (unsigned long)(-info->timer_delay);
		}
#ifdef CONFIG_ARM_ARCH_TIMER
		if (info->blow_data_correction) {
			info->delay_cycles[index] = __timer_delay_get_count(info->low_delay[index]);
			if (info->low_delay[index] > 2000000)
				info->ktime_delay[index] = ktime_set(0, (calc_low_delay * 2 / 3));		// note4(qualcomm) customizing : for edit!!
			else
				info->ktime_delay[index] = ktime_set(0, (calc_low_delay * 1 / 2));		// note4(qualcomm) customizing : for edit!!
		}
		else
#endif
			info->ktime_delay[index] = ktime_set(0, calc_low_delay);		// note4(qualcomm) customizing : for edit!!
	}
	/*********************************************/
	if (info->bfreq_fix) {
		ret = cpufreq_register_notifier(&notifier_policy_block,
							CPUFREQ_POLICY_NOTIFIER);
		if (ret != 0) {
			pr_err("[GPIO_IR][%s] cpufreq_register_notifier Error!\n", __func__);
			goto error_mutex_in;
		}
		GIR_LOG("[GPIO_IR][%s] cpu freq max fix\n", __func__);
	}
	if (info->bfreq_update) {
		j = 0;
		for_each_online_cpu(i) {
			cpufreq_update_policy(i);
			j++;
		}
	}
	/*********************************************/

	if (info->bpartial_timer)
	{
		info->data_pos = 0;
		info->bcompleted = false;
		
		if (info->partial_timer_type == 2)
			info->timer.function = gpio_ir_timeout_type2;
		else
			info->timer.function = gpio_ir_timeout_type1;
		GIR_TIMER_START(&info->timer, ktime_set(0, info->start_delay));

		wait_for_completion(&info->ir_send_comp);
	}
	else {
		pr_info("[GPIO_IR][%s] ir start!\n", __func__);
		GIR_LOCK(flags);

		gpio_ir_delay(info->start_delay);

		for (i = 0; i < info->count; i++) {
			if ((i & 0x01) == 0) {
				if (info->ir_data_type == 2)
					temp_count = (info->data[i] * 1000) /info->period;
				else
					temp_count = info->data[i];

#ifdef CONFIG_ARM_ARCH_TIMER
				GPIO_IR_SET_VALUE(0);
				start_cycle = get_cycles();
				if (info->bhigh_data_correction) {
					info->high_start_cycle = start_cycle;
				}
				for (i = 0; i < temp_count; i++) {
					GPIO_IR_SET_VALUE(1);
					start_cycle = __gpio_ir_timer_delay_from_start(start_cycle, info->up_delay_cycles);
					GPIO_IR_SET_VALUE(0);
					start_cycle = __gpio_ir_timer_delay_from_start(start_cycle, info->down_delay_cycles);
					if (info->bhigh_data_correction && 
							((start_cycle - info->high_start_cycle) >= info->delay_cycles[i])) {
						GIR_LOG("[GPIO_IR][%s] High Data Correcton! (%d:%d)\n", 
							__func__, temp_count, i);
						break;
					}
				}
#else
				for (j = 0; j < temp_count; j++) {
					GPIO_IR_SET_VALUE(1);
					gpio_ir_new_ndelay(info->up_delay);
					GPIO_IR_SET_VALUE(0);
					gpio_ir_new_ndelay(info->down_delay);
				}
#endif
			}
			else {
				gpio_ir_delay(info->low_delay[i]);
			}
		}		

		gpio_ir_delay(info->end_delay);
		GIR_UNLOCK(flags);
		pr_info("[GPIO_IR][%s] ir end!\n", __func__);
	}
	
	/*********************************************/
	if (info->bfreq_fix) {
		ret = cpufreq_unregister_notifier(&notifier_policy_block,
							CPUFREQ_POLICY_NOTIFIER);
		if (ret != 0) {
			pr_err("[GPIO_IR][%s] cpufreq_unregister_notifier Error!\n", __func__);
			goto error_mutex_in;
		}

		GIR_LOG("[GPIO_IR][%s] cpu freq max fix restore\n", __func__);
	}
	irled_power_onoff(0, info->en_gpio);

	if (info->blpm_disable) {
		pm_qos_update_request(&info->pm_qos_req, PM_QOS_DEFAULT_VALUE);
	}
	info->check_ir_send = 1;			//send success
error_mutex_in:
	mutex_unlock(&info->mutex_lock);

//error:
	GIR_LOG("[GPIO_IR][%s] --(%d)\n", __func__, ret);
	return;
}
	
#ifdef CONFIG_OF
int ir_gpio_parse_dt(struct device_node *node, struct gpio_ir_info_t *info)
{
	struct device_node *np = node;
	int gpiopin;
	int en_gpiopin;
	int ret;
	
	gpiopin = of_get_named_gpio(np, "samsung,irda_fet_control", 0);	

	ret = of_property_read_u32(np, "samsung,ir_led_en_gpio", &en_gpiopin);
	if (ret < 0) {
		pr_err("[%s]: no ir_led_en \n", __func__);
		info->en_gpio = NO_PIN_DETECTED;
		goto error;
	}
	info->en_gpio = en_gpiopin;
	info->gpio = gpiopin;
	
	return 1;
error:
	return -ENODEV;
}
#endif
static ssize_t remocon_ack(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct gpio_ir_info_t *info = dev_get_drvdata(dev);

	if (info->check_ir_send)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

static ssize_t remocon_show	(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gpio_ir_info_t *info = dev_get_drvdata(dev);

	int i;
	char *bufp = buf;

	bufp += sprintf(bufp, "%u,", info->freq);

#ifndef GPIO_DATA_REALTIME_ALLOC
	for (i = 0; i < IR_DATA_SIZE; i++) {
		if (info->data[i] == 0)
			break;
		else
			bufp += sprintf(bufp, "%u,", info->data[i]);		
	}
#endif

	return strlen(buf);
}

	
static ssize_t remocon_store(
	struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpio_ir_info_t *info = dev_get_drvdata(dev);

	int ret = 0;
	int i = 0;
	long temp_num;

	char *sep = ",";
	char *tok = NULL;
	char *string;

	GIR_LOG("[GPIO_IR][%s] ++\n", __func__);
	GIR_LOG("[GPIO_IR][%s] buf: %s\n", __func__, buf);

	string = kstrdup(buf, GFP_KERNEL);
	if (IS_ERR(string))
		return PTR_ERR(string);

#ifdef GPIO_DATA_REALTIME_ALLOC
	info->data = devm_kzalloc(dev, sizeof(unsigned int) * IR_DATA_SIZE, GFP_KERNEL);
	info->low_delay = devm_kzalloc(dev, sizeof(unsigned long) * IR_DATA_SIZE, GFP_KERNEL);
	info->ktime_delay = devm_kzalloc(dev, sizeof(ktime_t) * IR_DATA_SIZE, GFP_KERNEL);
	info->delay_cycles = devm_kzalloc(dev, sizeof(unsigned long) * IR_DATA_SIZE, GFP_KERNEL);
	if (!info->data || !info->low_delay || !info->ktime_delay ||! info->delay_cycles) {
		pr_err("Failed to allocate irda array!!");
		size = -ENOMEM;
		goto error;
	}
#endif
	memset(info->data, 0, sizeof(unsigned int) * IR_DATA_SIZE);
	info->count  = 0;
	info->freq = 0;

	for (i = 0; i < IR_DATA_SIZE+1; info->count = i++) {
		if ((tok = strsep(&string, sep)) == NULL)
			break;

		if ((ret = kstrtol(tok, 10, &temp_num)) < 0) {	//str to long returns 0 if success
			size = ret;
			goto error;
		}

		if (i == 0)
			info->freq = temp_num;
		else
			info->data[i-1] = temp_num;
	}

	if (info->freq > 100000) {		// for bang & oulfsen
		info->freq = 62700;
	}
	GIR_LOG("[GPIO_IR][%s] data count : %d\n", __func__, info->count);

	gpio_ir_remote_write(info);


error:
#ifdef GPIO_DATA_REALTIME_ALLOC
		if (info->data) {
			devm_kfree(dev, info->data);
			info->data = NULL;
		}
		if (info->low_delay) {
			devm_kfree(dev, info->low_delay);
			info->low_delay = NULL;
		}
		if (info->ktime_delay) {
			devm_kfree(dev, info->ktime_delay);
			info->low_delay = NULL;
		}
		if (info->delay_cycles) {
			devm_kfree(dev, info->delay_cycles);
			info->delay_cycles = NULL;
		}
#endif

	GIR_LOG("[GPIO_IR][%s] --(%d)\n", __func__, size);
	return size;
} //end of remocon_store


extern struct class *sec_class;
static int gpio_ir_probe(struct platform_device *pdev)
{
	int ret = 0;

	struct device *gpio_ir_dev;
	struct gpio_ir_info_t *info;

	pr_info("[GPIO_IR] %s has been created!!!\n", __func__);

	info = devm_kzalloc(&pdev->dev, sizeof(struct gpio_ir_info_t), GFP_KERNEL);
	if (!info) {
		pr_err("Failed to allocate memory for gpio ir\n");
		ret = -ENOMEM;
		goto fail_out;
	}	

	gpio_ir_dev = device_create(sec_class, NULL, 0, NULL, "sec_ir");
	if (IS_ERR(gpio_ir_dev)) {
		ret = PTR_ERR(gpio_ir_dev);
	
		pr_err("Failed to create device(gpio_ir)");
		goto fail_out;
	}

	dev_set_drvdata(gpio_ir_dev, info);

	ret = sysfs_create_group(&gpio_ir_dev->kobj, &gpio_ir_attr_group);
	if (ret) {
		pr_err("Failed to create sysfs group");
	
		goto fail_out;
	}
	
#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		ir_gpio_parse_dt(pdev->dev.of_node, info);		
	}
#else
	info->en_gpio = GPIO_IR_LED_EN;
	info->gpio = GPIO_IRLED_PIN;
#endif
	/*****************************************************/
	/* Tunning Data Init									  */
	info->bfreq_fix = true;
	info->bfreq_update = true;
	info->blpm_disable = true;
	info->bpartial_timer = true;
	info->partial_timer_type = 1;
	info->bir_data_type_auto = false;
	info->ir_data_type = 2;
	info->gpio_high_delay = 400;
	info->gpio_low_delay = 100;
	info->gpio_custom_delay = 0;
	info->start_delay = 60000000;
	info->end_delay = 13000000;
	info->timer_delay = 29670;
#ifdef CONFIG_ARM_ARCH_TIMER
	info->bhigh_data_correction = true;
	info->blow_data_correction = true;
#endif
	/*****************************************************/
	
	gpio_request_one(info->gpio, GPIOF_OUT_INIT_LOW, "IRLED");

	// for data init & debug
	mutex_init(&info->mutex_lock);
	
#ifdef GPIO_IR_PARTIAL_TIMER
	GIR_TIMER_INIT(&info->timer);
#endif
	init_completion(&info->ir_send_comp);

#ifndef GPIO_DATA_REALTIME_ALLOC
	info->data = devm_kzalloc(&pdev->dev, sizeof(unsigned int) * IR_DATA_SIZE, GFP_KERNEL);
	info->low_delay = devm_kzalloc(&pdev->dev, sizeof(unsigned long) * IR_DATA_SIZE, GFP_KERNEL);
	info->ktime_delay = devm_kzalloc(&pdev->dev, sizeof(ktime_t) * IR_DATA_SIZE, GFP_KERNEL);
	info->delay_cycles = devm_kzalloc(&pdev->dev, sizeof(unsigned long) * IR_DATA_SIZE, GFP_KERNEL);
	if (!info->data || !info->low_delay || !info->ktime_delay ||! info->delay_cycles) {
		pr_err("Failed to allocate irda arrays!!");
		ret = -ENOMEM;
		goto fail_out;
	}
#endif

	if (info->blpm_disable)
		pm_qos_add_request(&info->pm_qos_req,
						PM_QOS_CPU_DMA_LATENCY,
						PM_QOS_DEFAULT_VALUE);	

	irled_power_init(info->en_gpio);

	return 0;
fail_out:
	if (ret) {
		pr_err(" (err = %d)!\n", ret);
	}	
	
	return ret;
};

static int gpio_ir_remove(struct platform_device *pdev)
{
	struct gpio_ir_info_t *info = dev_get_platdata(&pdev->dev);

#ifndef GPIO_DATA_REALTIME_ALLOC
	if (info->data) {
		devm_kfree(&pdev->dev, info->data);
		info->data = NULL;
	}
	if (info->low_delay) {
		devm_kfree(&pdev->dev, info->low_delay);
		info->low_delay = NULL;
	}
	if (info->ktime_delay) {
		devm_kfree(&pdev->dev, info->ktime_delay);
		info->low_delay = NULL;
	}
	if (info->delay_cycles) {
		devm_kfree(&pdev->dev, info->delay_cycles);
		info->delay_cycles = NULL;
	}
#endif
	return 0;
}

static const struct of_device_id  irda_gpio_id[] = {
	{.compatible = "gpio_ir"}
};

static struct platform_driver gpio_ir = {
	.probe = gpio_ir_probe,
	.remove = gpio_ir_remove,
	.driver = {
		.name = "gpio_ir",
		.owner = THIS_MODULE,
		.of_match_table = irda_gpio_id,
	},
};

static int __init gpio_ir_init(void)
{
	int ret;
	ret = platform_driver_register(&gpio_ir);
	pr_info("[GPIO_IR][%s] initialized!!! %d\n", __func__, ret);
	return ret;
}

static void __exit gpio_ir_exit(void)
{
	platform_driver_unregister(&gpio_ir);
}

module_init(gpio_ir_init);
module_exit(gpio_ir_exit);

MODULE_AUTHOR("inate.jun@samsung.com, jeeon.park@samsung.com, yonghune.an@samsung.com");
MODULE_DESCRIPTION("Samsung Electronics Co. IRDA using GPIO module");
MODULE_LICENSE("GPL");


