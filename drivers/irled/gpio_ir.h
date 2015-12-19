/*
  * Samsung Mobile VE Group.
  *
  * drivers/irled/gpio_ir.h
  *
  * Header for samsung gpio irled.
  *
  * Copyright (c) 2014 Samsung Electronics Co., Ltd.
  *  Jeeon Park <jeeon.park@samsung.com>
  *
  * This program is free software. You can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation
  */

#ifndef _GPIO_IR_H_
#define _GPIO_IR_H_

/****************************************************************/
/* Define this feature in accordance with the state of each BB platform                */

//#define GPIO_IR_CUSTOM_LOCK_EXYNOS	// for exynos model
#define GPIO_IR_CUSTOM_NDELAY
#define GPIO_IR_CPU_FREQ_FIX
#define GPIO_IR_LPM_DISABLE_QUALCOMM
#define GPIO_IR_CPUBOOST_DISABLE_QUALCOMM
#define GPIO_IR_PARTIAL_TIMER
/********************************/
#ifdef GPIO_IR_PARTIAL_TIMER
#if 0
#define GPIO_IR_TIMER1
//#define GPIO_IR_TIMER2
//#define GPIO_IR_TIMER3
//#define GPIO_IR_TIMER4
//#define GPIO_IR_TIMER5
//#define GPIO_IR_TIMER6
#else
#define GPIO_IR_TIMER_SELECT
#endif

#define GPIO_IR_TIMER_TYPE1
//#define GPIO_IR_TIMER_TYPE2

#if defined(GPIO_IR_TIMER_TYPE1) && defined(GPIO_IR_TIMER_TYPE2)
#error "[Error][GPIO_IR] Duplicate TIMER METHOD feature!"
#endif
#if !defined(GPIO_IR_TIMER_TYPE1) && !defined(GPIO_IR_TIMER_TYPE2)
#error "[Error][GPIO_IR] NO define TIMER METHOD feature!"
#endif

#define IR_TIMER_CALL_DELAY	29670	//20000	/* This value may be changed in each models */
#endif	// #ifdef GPIO_IR_PARTIAL_TIMER
/********************************/
//#define GPIO_IR_DATA_TYPE1
#define GPIO_IR_DATA_TYPE2
#define GPIO_IR_DATA_TYPE_AUTO

#if defined(GPIO_IR_DATA_TYPE1) && defined(GPIO_IR_DATA_TYPE2)
#error "[Error][GPIO_IR] Duplicate DATA TYPE feature!"
#endif
#if !defined(GPIO_IR_DATA_TYPE1) && !defined(GPIO_IR_DATA_TYPE2)
#error "[Error][GPIO_IR] No define DATA TYPE feature!"
#endif

/********************************/
#define GIR_DATA_START_DELAY	30000000
#define GIR_DATA_END_DELAY		30000000
/********************************/
#define GIR_DIV_LOOP_COUNT	1220
/****************************************************************/

// for debugging by reading ir_send, disable this define.
//#define GPIO_DATA_REALTIME_ALLOC

// for debugging by ir test app, enable this define.
#define GPIO_IR_DEBUG

#ifdef GPIO_IR_CUSTOM_LOCK_EXYNOS
#include <plat/gpio-core.h>
#endif


//#define PJE_DELAY_TEST

//#define	GPIO_IRLED_PIN		GPIO_MAIN_CAM_RESET		// need to customize
//#define	GPIO_IRLED_PIN		38	// need to customize
#define	NO_PIN_DETECTED		-1

#define GPIO_LEVEL_LOW	0
#define GPIO_LEVEL_HIGH	1

#ifndef CONFIG_OF
#define GPIO_IR_LED_EN	301	/* If device did not use dts, This value is used. */
#define GPIO_IRLED_PIN	38	
#endif

#define IR_DATA_SIZE	1024
#define IR_FREQ_UNIT_MHZ		1000000
#define IR_FREQ_UNIT_GHZ		1000000000UL

//#define IR_GPIO_DELAY	200			/* This value may be changed in each models */
#define IR_GPIO_HIGH_DELAY	400			/* This value may be changed in each models */
#define IR_GPIO_LOW_DELAY	100			/* This value may be changed in each models */
#define IR_GPIO_CUSTOM_DELAY	0		/* This value may be changed in each models */

struct gpio_ir_info_t
{
	int gpio;
	int en_gpio;
	unsigned int freq;
	unsigned int *data;
	int count;
	struct mutex mutex_lock;
	unsigned long up_delay;
	unsigned long down_delay;
#ifdef CONFIG_ARM_ARCH_TIMER
	unsigned long up_delay_cycles;
	unsigned long down_delay_cycles;
#endif
	unsigned long period;
	
	// partial timer
	struct hrtimer timer;
	struct completion ir_send_comp;
	int data_pos;

	bool bfreq_fix;
	bool bfreq_update;
	bool blpm_disable;
	bool bpartial_timer;
	unsigned long partial_timer_type;
	bool bir_data_type_auto;
	unsigned long ir_data_type;	
	unsigned long gpio_high_delay;
	unsigned long gpio_low_delay;
	long gpio_custom_delay;
	unsigned long start_delay;
	unsigned long end_delay;
	long timer_delay;
#ifdef CONFIG_ARM_ARCH_TIMER
	bool bhigh_data_correction;		// high data length correction
	bool blow_data_correction;		// low data length correction
#endif

	unsigned long *low_delay;
	ktime_t *ktime_delay;
#ifdef CONFIG_ARM_ARCH_TIMER
	unsigned long *delay_cycles;
	unsigned long high_start_cycle;
	unsigned long low_start_cycle;
#endif
	bool bcompleted;
	bool check_ir_send;
	
	struct pm_qos_request pm_qos_req;
};

//extern cycles_t __timer_delay_pje(cycles_t start, unsigned long cycles);
//extern unsigned long __timer_delay_pje_count(unsigned long nsecs);
//extern int cpufreq_cpu_get_freq(unsigned int *min, unsigned int *max, unsigned int *cur);
extern unsigned int __loop_custom_ndelay(unsigned long nsecs);
#endif

