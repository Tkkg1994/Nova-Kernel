/*
 *  drivers/cpufreq/cpufreq_HYPER.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 *                 2012 Minor Edits by Sar Castillo <sar.castillo@gmail.com>
 *                 2012 MAR heavy addons by DORIMANX <yuri@bynet.co.il>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/slab.h>

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define MIN_FREQUENCY_DOWN_DIFFERENTIAL		(1)
#define DEF_FREQUENCY_UP_THRESHOLD		(70)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define BOOSTED_SAMPLING_DOWN_FACTOR		(10)
#define MAX_SAMPLING_DOWN_FACTOR		(3)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(5)
#define MICRO_FREQUENCY_UP_THRESHOLD		(70)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(5000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define MIN_FREQUENCY_DOWN_DIFFERENTIAL		(1)
#define FREQ_STEP				(50)
#define DEFAULT_FREQ_BOOST_TIME			(500000)
#define MAX_FREQ_BOOST_TIME			(5000000)
#define UP_THRESHOLD_AT_MIN_FREQ		(40)

static u64 hyper_freq_boosted_time;

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */
#define MIN_SAMPLING_RATE_RATIO			(2)

static unsigned int min_sampling_rate;
#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

/* have the timer rate booted for this much time 4s*/
#define TIMER_RATE_BOOST_TIME 4000000
static int hyper_sampling_rate_boosted;
static u64 hyper_sampling_rate_boosted_time;
unsigned int hyper_current_sampling_rate;

static void do_dbs_timer(struct work_struct *work);

/* Sampling types */
enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

struct cpu_dbs_info_s {
	u64 prev_cpu_idle;
	u64 prev_cpu_iowait;
	u64 prev_cpu_wall;
	unsigned int prev_cpu_wall_delta;
	u64 prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_lo;
	unsigned int freq_lo_jiffies;
	unsigned int freq_hi_jiffies;
	unsigned int rate_mult;
	unsigned int load_at_prev_sample;
	unsigned int cpu;
	unsigned int sample_type:1;
	unsigned int prev_load;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, od_cpu_dbs_info);

static unsigned int dbs_enable;	/* number of CPUs using this policy */

/*
 * dbs_mutex protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);

static struct workqueue_struct *dbs_wq;

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int up_threshold_min_freq;
	unsigned int down_differential;
	unsigned int micro_freq_up_threshold;
	unsigned int sampling_down_factor;
	unsigned int boosted;
	unsigned int freq_boost_time;
	unsigned int boostfreq;
	/*struct notifier_block dvfs_lat_qos_db;
	unsigned int dvfs_lat_qos_wants;*/
	unsigned int freq_step;
} dbs_tuners_ins = {
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.up_threshold_min_freq = UP_THRESHOLD_AT_MIN_FREQ,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.micro_freq_up_threshold = MICRO_FREQUENCY_UP_THRESHOLD,
	.freq_boost_time = DEFAULT_FREQ_BOOST_TIME,
	.boostfreq = 1728000,
	.freq_step = FREQ_STEP,
	.sampling_rate = 20000,
};

static unsigned int dbs_enable = 0;	/* number of CPUs using this policy */

static inline u64 get_cpu_iowait_time(unsigned int cpu, u64 *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

/************************** sysfs interface ************************/

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_min);

/* cpufreq_hyper_power Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)              \
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(up_threshold, up_threshold);
show_one(up_threshold_min_freq, up_threshold_min_freq);
show_one(sampling_down_factor, sampling_down_factor);
show_one(micro_freq_up_threshold, micro_freq_up_threshold);
show_one(down_differential, down_differential);
show_one(boostpulse, boosted);
show_one(boostfreq, boostfreq);
show_one(freq_step, freq_step);

static ssize_t store_boostpulse(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned int input;

	ret = sscanf(buf, "%u", &input);
	if (ret < 0)
		return ret;

	if (input > 1 && input <= MAX_FREQ_BOOST_TIME)
		dbs_tuners_ins.freq_boost_time = input;
	else
		dbs_tuners_ins.freq_boost_time = DEFAULT_FREQ_BOOST_TIME;

	dbs_tuners_ins.boosted = 1;
	hyper_freq_boosted_time = ktime_to_us(ktime_get());

	if (hyper_sampling_rate_boosted) {
		hyper_sampling_rate_boosted = 0;
		dbs_tuners_ins.sampling_rate = hyper_current_sampling_rate;
	}
	return count;
}

static ssize_t store_boostfreq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.boostfreq = input;
	return count;
}

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret = 0;
	int mpd = strcmp(current->comm, "mpdecision");

	if (mpd == 0)
		return ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.sampling_rate = max(input, min_sampling_rate);
	hyper_current_sampling_rate = dbs_tuners_ins.sampling_rate;

	return count;
}

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.up_threshold = input;

	return count;
}

static ssize_t store_micro_freq_up_threshold(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.micro_freq_up_threshold = input;
	return count;
}

static ssize_t store_down_differential(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.down_differential = min(input, 100u);

	return count;
}

static ssize_t store_up_threshold_min_freq(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.up_threshold_min_freq = input;

	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;

	dbs_tuners_ins.sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->rate_mult = 1;
	}
	return count;
}

static ssize_t store_freq_step(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.freq_step = min(input, 100u);
	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(up_threshold);
define_one_global_rw(up_threshold_min_freq);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(down_differential);
define_one_global_rw(micro_freq_up_threshold);
define_one_global_rw(boostpulse);
define_one_global_rw(boostfreq);
define_one_global_rw(freq_step);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
	&up_threshold_min_freq.attr,
	&sampling_down_factor.attr,
	&down_differential.attr,
	&micro_freq_up_threshold.attr,
	&boostpulse.attr,
	&boostfreq.attr,
	&freq_step.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "HYPER",
};

/************************** sysfs end ************************/

static void dbs_freq_increase(struct cpufreq_policy *p, unsigned int freq)
{
	if (p->cur == p->max)
		return;

	__cpufreq_driver_target(p, freq, CPUFREQ_RELATION_L);
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	/* Extrapolated load of this CPU */
	unsigned int load_at_max_freq = 0;
	unsigned int avg_load_at_max_freq = 0;
	unsigned int max_load_freq;
	/* Current load across this CPU */
	unsigned int cur_load = 0;
	unsigned int max_load = 0;

	struct cpufreq_policy *policy;
	struct cpu_dbs_info_s *j_dbs_info;
	unsigned int j = 0;
	int up_threshold = dbs_tuners_ins.up_threshold;
	unsigned int sampling_rate;

	sampling_rate = dbs_tuners_ins.sampling_rate * this_dbs_info->rate_mult;
	this_dbs_info->freq_lo = 0;

	policy = this_dbs_info->cur_policy;
	if (policy == NULL)
		return;

	/* Only core0 controls the boost */
	if (dbs_tuners_ins.boosted && policy->cpu == 0) {
		if (ktime_to_us(ktime_get()) - hyper_freq_boosted_time >=
					dbs_tuners_ins.freq_boost_time) {
			dbs_tuners_ins.boosted = 0;
		}
	}

	/* Only core0 controls the timer_rate */
	if (hyper_sampling_rate_boosted && policy->cpu == 0) {
		if (ktime_to_us(ktime_get()) - hyper_sampling_rate_boosted_time >=
					TIMER_RATE_BOOST_TIME) {

			dbs_tuners_ins.sampling_rate = hyper_current_sampling_rate;
			hyper_sampling_rate_boosted = 0;
		}
	}

	/*
	 * Every sampling_rate, we check, if current idle time is less
	 * than 20% (default), then we try to increase frequency
	 * Every sampling_rate, we look for a the lowest
	 * frequency which can sustain the load while keeping idle time over
	 * 30%. If such a frequency exist, we try to decrease to this frequency.
	 *
	 * Any frequency increase takes it to the maximum frequency.
	 * Frequency reduction happens at minimum steps of
	 * 5% (default) of current frequency
	 */

	/* Get Absolute Load - in terms of freq */
	max_load_freq = 0;

	for_each_cpu(j, policy->cpus) {
		u64 cur_wall_time, cur_idle_time, cur_iowait_time;
		unsigned int idle_time, wall_time, iowait_time;
		unsigned int load_freq;
		int freq_avg;
		bool deep_sleep_detected = false;
		/* the evil magic numbers, only 2 at least */
		const unsigned int deep_sleep_backoff = 10;
		const unsigned int deep_sleep_factor = 5;

		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time, 0);
		cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

		wall_time = (unsigned int)
			(cur_wall_time - j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		/*
		 * Ignore wall delta jitters in both directions.  An
		 * exceptionally long wall_time will likely result
		 * idle but it was waken up to do work so the next
		 * slice is less likely to want to run at low
		 * frequency. Let's evaluate the next slice instead of
		 * the idle long one that passed already and it's too
		 * late to reduce in frequency.  As opposed an
		 * exceptionally short slice that just run at low
		 * frequency is unlikely to be idle, but we may go
		 * back to idle pretty soon and that not idle slice
		 * already passed. If short slices will keep coming
		 * after a series of long slices the exponential
		 * backoff will converge faster and we'll react faster
		 * to high load. As opposed we'll decay slower
		 * towards low load and long idle times.
		 */
		if (j_dbs_info->prev_cpu_wall_delta >
		    wall_time * deep_sleep_factor ||
		    j_dbs_info->prev_cpu_wall_delta * deep_sleep_factor <
		    wall_time)
			deep_sleep_detected = true;
		j_dbs_info->prev_cpu_wall_delta =
			(j_dbs_info->prev_cpu_wall_delta * deep_sleep_backoff
			 + wall_time) / (deep_sleep_backoff+1);

		idle_time = (unsigned int)
			(cur_idle_time - j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		iowait_time = (unsigned int)
			(cur_iowait_time - j_dbs_info->prev_cpu_iowait);
		j_dbs_info->prev_cpu_iowait = cur_iowait_time;

		if (deep_sleep_detected)
			continue;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		/*
		 * If the CPU had gone completely idle, and a task just woke up
		 * on this CPU now, it would be unfair to calculate 'load' the
		 * usual way for this elapsed time-window, because it will show
		 * near-zero load, irrespective of how CPU intensive that task
		 * actually is. This is undesirable for latency-sensitive bursty
		 * workloads.
		 *
		 * To avoid this, we reuse the 'load' from the previous
		 * time-window and give this task a chance to start with a
		 * reasonably high CPU frequency. (However, we shouldn't over-do
		 * this copy, lest we get stuck at a high load (high frequency)
		 * for too long, even when the current system load has actually
		 * dropped down. So we perform the copy only once, upon the
		 * first wake-up from idle.)
		 *
		 * Detecting this situation is easy: the governor's deferrable
		 * timer would not have fired during CPU-idle periods. Hence
		 * an unusually large 'wall_time' (as compared to the sampling
		 * rate) indicates this scenario.
		 *
		 * prev_load can be zero in two cases and we must recalculate it
		 * for both cases:
		 * - during long idle intervals
		 * - explicitly set to zero
		 */
		if (unlikely(wall_time > (2 * sampling_rate) &&
			     j_dbs_info->prev_load)) {
			cur_load = j_dbs_info->prev_load;

			/*
			 * Perform a destructive copy, to ensure that we copy
			 * the previous load only once, upon the first wake-up
			 * from idle.
			 */
			j_dbs_info->prev_load = 0;
		} else {
			cur_load = 100 * (wall_time - idle_time) / wall_time;
			j_dbs_info->prev_load = cur_load;
		}

		if (cur_load > max_load)
			max_load = cur_load;

		freq_avg = __cpufreq_driver_getavg(policy, j);
		if (freq_avg <= 0)
			freq_avg = policy->cur;

		load_freq = cur_load * freq_avg;
		if (load_freq > max_load_freq)
			max_load_freq = load_freq;

		/* calculate the scaled load across CPU */
		load_at_max_freq += (cur_load * policy->cur) /
					policy->max;

		avg_load_at_max_freq += ((load_at_max_freq +
				j_dbs_info->load_at_prev_sample) / 2);

		j_dbs_info->load_at_prev_sample = load_at_max_freq;
	}

	cpufreq_notify_utilization(policy, load_at_max_freq);

	/* Check for frequency increase */
	if (max_load_freq > up_threshold * policy->cur) {
		int inc = (policy->max * dbs_tuners_ins.freq_step) / 100;
		int target = min(policy->max, policy->cur + inc);

		/* If switching to max speed, apply sampling_down_factor */
		if (policy->cur < policy->max && target == policy->max) {
			if (hyper_sampling_rate_boosted &&
				(dbs_tuners_ins.sampling_down_factor <
					BOOSTED_SAMPLING_DOWN_FACTOR)) {
				this_dbs_info->rate_mult =
					BOOSTED_SAMPLING_DOWN_FACTOR;
			} else {
				this_dbs_info->rate_mult =
					dbs_tuners_ins.sampling_down_factor;
			}
		}
		dbs_freq_increase(policy, target);
		return;
	}

	/* check for frequency boost */
	if (dbs_tuners_ins.boosted && policy->cur < dbs_tuners_ins.boostfreq) {
		dbs_freq_increase(policy, dbs_tuners_ins.boostfreq);
		dbs_tuners_ins.boostfreq = policy->cur;
		return;
	}

	/* Check for frequency decrease */
	/* if we cannot reduce the frequency anymore, break out early */
	if (policy->cur == policy->min)
		return;

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy. To be safe, we focus 10 points under the threshold.
	 */
	if (max_load_freq <
	    (dbs_tuners_ins.up_threshold - dbs_tuners_ins.down_differential) *
			policy->cur) {
		unsigned int freq_next;
		unsigned int down_thres;

		freq_next = max_load_freq /
				(dbs_tuners_ins.up_threshold -
					dbs_tuners_ins.down_differential);

		if (dbs_tuners_ins.boosted &&
				freq_next < dbs_tuners_ins.boostfreq) {
			freq_next = dbs_tuners_ins.boostfreq;
		}
		/* No longer fully busy, reset rate_mult */
		this_dbs_info->rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

		down_thres = dbs_tuners_ins.up_threshold_min_freq
			- dbs_tuners_ins.down_differential;

		__cpufreq_driver_target(policy, freq_next,
			CPUFREQ_RELATION_L);
	}
}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	int sample_type = dbs_info->sample_type;

	int delay;

	if (unlikely(!cpu_online(dbs_info->cpu) || !dbs_info->cur_policy))
		return;

	mutex_lock(&dbs_info->timer_mutex);

	/* Common NORMAL_SAMPLE setup */
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	if (sample_type == DBS_NORMAL_SAMPLE) {
		dbs_check_cpu(dbs_info);
		if (dbs_info->freq_lo) {
			/* Setup timer for SUB_SAMPLE */
			dbs_info->sample_type = DBS_SUB_SAMPLE;
			delay = dbs_info->freq_hi_jiffies;
		} else {
			/* We want all CPUs to do sampling nearly on
			 * same jiffy
			 */
			delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate
				* dbs_info->rate_mult);

			if (num_online_cpus() > 1)
				delay -= jiffies % delay;
		}
	} else {
		__cpufreq_driver_target(dbs_info->cur_policy,
			dbs_info->freq_lo, CPUFREQ_RELATION_H);
		delay = dbs_info->freq_lo_jiffies;
	}
	queue_delayed_work_on(dbs_info->cpu, dbs_wq, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	INIT_DEFERRABLE_WORK(&dbs_info->work, do_dbs_timer);
	queue_delayed_work_on(dbs_info->cpu, dbs_wq, &dbs_info->work,
				10 * delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	cancel_delayed_work_sync(&dbs_info->work);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		dbs_enable++;

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			unsigned int prev_load;

			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall, 0);

			prev_load = (unsigned int)
				(j_dbs_info->prev_cpu_wall - j_dbs_info->prev_cpu_idle);
			j_dbs_info->prev_load = 100 * prev_load /
				(unsigned int) j_dbs_info->prev_cpu_wall;
		}
		cpu = policy->cpu;
		this_dbs_info->cpu = cpu;
		this_dbs_info->rate_mult = 1;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			unsigned int latency;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				dbs_enable--;
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;
			/* Bring kernel and HW constraints together */
			min_sampling_rate = max(min_sampling_rate,
					MIN_LATENCY_MULTIPLIER * latency);
			if (latency != 1)
				dbs_tuners_ins.sampling_rate =
					max(dbs_tuners_ins.sampling_rate,
						latency * LATENCY_MULTIPLIER);
		}
		mutex_unlock(&dbs_mutex);

		mutex_init(&this_dbs_info->timer_mutex);

		dbs_timer_init(this_dbs_info);

		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		mutex_destroy(&this_dbs_info->timer_mutex);

		dbs_enable--;
		if (!dbs_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);
		mutex_unlock(&dbs_mutex);

		break;

	case CPUFREQ_GOV_LIMITS:
		/* If device is being removed, skip set limits */
		if (!this_dbs_info->cur_policy)
			break;
		mutex_lock(&this_dbs_info->timer_mutex);
		__cpufreq_driver_target(this_dbs_info->cur_policy,
				policy->cur, CPUFREQ_RELATION_L);
		dbs_check_cpu(this_dbs_info);
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_HYPER
static
#endif
struct cpufreq_governor cpufreq_gov_HYPER = {
	.name			= "HYPER",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	u64 wall;
	u64 idle_time;
	int cpu = get_cpu();
	int err = 0;

	idle_time = get_cpu_idle_time_us(cpu, &wall);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		dbs_tuners_ins.up_threshold = dbs_tuners_ins.micro_freq_up_threshold;
		dbs_tuners_ins.down_differential =
					MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		/*
		 * In no_hz/micro accounting case we set the minimum frequency
		 * not depending on HZ, but fixed (very low). The deferred
		 * timer might skip some samples if idle/sleeping as needed.
		*/
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		/* For correct statistics, we need 10 ticks for each measure */
		min_sampling_rate =
			MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10);
	}

	dbs_wq = alloc_workqueue("HYPER_dbs_wq", WQ_HIGHPRI, 0);
	if (!dbs_wq) {
		printk(KERN_ERR "Failed to create HYPER_dbs_wq workqueue\n");
		return -EFAULT;
	}

	err = cpufreq_register_governor(&cpufreq_gov_HYPER);
	if (err)
		goto error_reg;

	return err;
error_reg:
	kfree(&dbs_tuners_ins);
	return err;
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	/*pm_qos_remove_notifier(PM_QOS_DVFS_RESPONSE_LATENCY,
			       &hyper_qos_dvfs_lat_nb);*/

	cpufreq_unregister_governor(&cpufreq_gov_HYPER);
	destroy_workqueue(dbs_wq);
	kfree(&dbs_tuners_ins);
}

MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_DESCRIPTION("'cpufreq_HYPER' - A dynamic cpufreq governor for "
	"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_HYPER
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
