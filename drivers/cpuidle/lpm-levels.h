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

#include <soc/qcom/pm.h>
#include <soc/qcom/spm.h>

#define NR_LPM_LEVELS 8

struct lpm_lookup_table {
	uint32_t modes;
	const char *mode_name;
};

struct power_params {
	uint32_t latency_us;		/* Enter + Exit latency */
	uint32_t ss_power;		/* Steady state power */
	uint32_t energy_overhead;	/* Enter + exit over head */
	uint32_t time_overhead_us;	/* Enter + exit overhead */
};

struct lpm_cpu_level {
	const char *name;
	enum msm_pm_sleep_mode mode;
	bool use_bc_timer;
	struct power_params pwr;
};

struct lpm_cpu {
	struct lpm_cpu_level levels[NR_LPM_LEVELS];
	int nlevels;
	struct lpm_cluster *parent;
};

struct lpm_cluster_level {
	const char *level_name;
	int *mode;			/* SPM mode to enter */
	int min_child_level;
	struct cpumask num_cpu_votes;
	struct power_params pwr;
	bool notify_rpm;
	bool available;
	bool sync_level;
	bool last_core_only;
};

struct low_power_ops {
	struct msm_spm_device *spm;
	int (*set_mode)(struct low_power_ops *ops, int mode, bool notify_rpm);
	enum msm_pm_l2_scm_flag tz_flag;
};

struct lpm_cluster {
	struct list_head list;
	struct list_head child;
	const char *cluster_name;
	const char **name;
	struct low_power_ops *lpm_dev;
	int ndevices;
	struct lpm_cluster_level levels[NR_LPM_LEVELS];
	int nlevels;
	enum msm_pm_l2_scm_flag l2_flag;
	int min_child_level;
	int default_level;
	int last_level;
	struct lpm_cpu *cpu;
	struct cpuidle_driver *drv;
	spinlock_t sync_lock;
	struct cpumask child_cpus;
	struct cpumask num_childs_in_sync;
	struct lpm_cluster *parent;
	struct lpm_stats *stats;
	bool no_saw_devices;
};

int set_l2_mode(struct low_power_ops *ops, int mode, bool notify_rpm);
int set_cci_mode(struct low_power_ops *ops, int mode, bool notify_rpm);

struct lpm_cluster *lpm_of_parse_cluster(struct platform_device *pdev);
void free_cluster_node(struct lpm_cluster *cluster);
void cluster_dt_walkthrough(struct lpm_cluster *cluster);
