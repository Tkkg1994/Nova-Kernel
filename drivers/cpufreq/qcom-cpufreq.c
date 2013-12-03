/* drivers/cpufreq/qcom-cpufreq.c
 *
 * MSM architecture cpufreq driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2015, The Linux Foundation. All rights reserved.
 * Author: Mike A. Chan <mikechan@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <trace/events/power.h>
#include <linux/msm-bus.h>

#ifdef CONFIG_CPU_VOLTAGE_CONTROL
static struct cpufreq_frequency_table *dts_freq_table;
#endif

static DEFINE_MUTEX(l2bw_lock);

static struct clk *cpu_clk[NR_CPUS];
static struct clk *l2_clk;
static DEFINE_PER_CPU(struct cpufreq_frequency_table *, freq_table);
static bool hotplug_ready;

static struct msm_bus_vectors *bus_vec_lst;
static struct msm_bus_scale_pdata bus_bw = {
	.name = "msm-cpufreq",
	.active_only = 1,
};

static u32 bus_client;

struct delayed_work	cpufreq_bimc_work;

static unsigned int boost_ms = 20;
module_param(boost_ms, uint, 0644);

struct cpufreq_work_struct {
	struct work_struct work;
	struct cpufreq_policy *policy;
	struct completion complete;
	int frequency;
	unsigned int index;
	int status;
};

static DEFINE_PER_CPU(struct cpufreq_work_struct, cpufreq_work);
static struct workqueue_struct *msm_cpufreq_wq;

struct cpufreq_suspend_t {
	struct mutex suspend_mutex;
	int device_suspended;
};

/* Max frequency to add to the frequency_table */
static unsigned long arg_cpu_max_freq = 2880000;

static int __init cpufreq_read_cpu_max_freq(char *cpu_max_freq)
{
	unsigned long ui_khz;
	int ret;

	ret = kstrtoul(cpu_max_freq, 0, &ui_khz);
	if (ret)
		return -EINVAL;

	arg_cpu_max_freq = ui_khz;
	printk("cpu_max_freq=%lu\n", arg_cpu_max_freq);
	return ret;
}
__setup("cpu_max_freq=", cpufreq_read_cpu_max_freq);

static DEFINE_PER_CPU(struct cpufreq_suspend_t, cpufreq_suspend);

static void cpufreq_bimc_release(struct work_struct *work)
{
	pr_debug("CPUFREQ: release BIMC voting\n");
	if (bus_client)
		msm_bus_scale_client_update_request(bus_client, 0);
}

extern int jig_boot_clk_limit;

static int set_cpu_freq(struct cpufreq_policy *policy, unsigned int new_freq,
			unsigned int index)
{
	int ret = 0;
	struct cpufreq_freqs freqs;
	unsigned long rate;

	freqs.old = policy->cur;
	freqs.new = new_freq;
	freqs.cpu = policy->cpu;

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

	trace_cpu_frequency_switch_start(freqs.old, freqs.new, policy->cpu);

	rate = new_freq * 1000;
#ifdef CONFIG_SEC_PM
#if defined(CONFIG_SEC_TRLTE_CHNDUOS) || defined(CONFIG_MACH_TRLTE_LDU) || defined(CONFIG_MACH_TBLTE_LDU)
#define JIG_LIMIT_CLK	1574400 * 1000
#define JIG_LIMIT_TIME	300
#else
#define JIG_LIMIT_CLK	1574400 * 1000
#define JIG_LIMIT_TIME	100
#endif
	if (jig_boot_clk_limit == 1) { //limit 1.5Ghz to block whitescreen during 50 secs on JIG
		unsigned long long t = sched_clock();
		do_div(t, 1000000000);
		if (t <= JIG_LIMIT_TIME && rate > JIG_LIMIT_CLK)
			rate = JIG_LIMIT_CLK;
		else if (t > JIG_LIMIT_TIME) {		// no need to check after 50 sec
			jig_boot_clk_limit = 0;
		}
	}
#endif
	rate = clk_round_rate(cpu_clk[policy->cpu], rate);
	ret = clk_set_rate(cpu_clk[policy->cpu], rate);
	if (!ret) {
		cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);
		trace_cpu_frequency_switch_end(policy->cpu);
	}

	return ret;
}

static void set_cpu_work(struct work_struct *work)
{
	struct cpufreq_work_struct *cpu_work =
		container_of(work, struct cpufreq_work_struct, work);

	cpu_work->status = set_cpu_freq(cpu_work->policy, cpu_work->frequency,
					cpu_work->index);
	complete(&cpu_work->complete);
}

static int msm_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	int ret = 0;
	int index;
	struct cpufreq_frequency_table *table;

	struct cpufreq_work_struct *cpu_work = NULL;

	mutex_lock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);

	if (target_freq == policy->cur)
		goto done;

	if (per_cpu(cpufreq_suspend, policy->cpu).device_suspended) {
		pr_debug("cpufreq: cpu%d scheduling frequency change "
				"in suspend.\n", policy->cpu);
		ret = -EFAULT;
		goto done;
	}

	table = cpufreq_frequency_get_table(policy->cpu);
	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
			&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		ret = -EINVAL;
		goto done;
	}

	pr_debug("CPU[%d] target %d relation %d (%d-%d) selected %d\n",
		policy->cpu, target_freq, relation,
		policy->min, policy->max, table[index].frequency);

	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	cpu_work->policy = policy;
	cpu_work->frequency = table[index].frequency;
	cpu_work->index = table[index].driver_data;
	cpu_work->status = -ENODEV;

	cancel_work_sync(&cpu_work->work);
	INIT_COMPLETION(cpu_work->complete);
	queue_work_on(policy->cpu, msm_cpufreq_wq, &cpu_work->work);
	wait_for_completion(&cpu_work->complete);

	ret = cpu_work->status;

done:
	mutex_unlock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);
	return ret;
}

static int msm_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
			policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int msm_cpufreq_get_freq(unsigned int cpu)
{
	return clk_get_rate(cpu_clk[cpu]) / 1000;
}

static int msm_cpufreq_init(struct cpufreq_policy *policy)
{
	int cur_freq;
	int index;
	int ret = 0;
	struct cpufreq_frequency_table *table =
			per_cpu(freq_table, policy->cpu);
	struct cpufreq_work_struct *cpu_work = NULL;
	int cpu;

	/*
	 * In some SoC, some cores are clocked by same source, and their
	 * frequencies can not be changed independently. Find all other
	 * CPUs that share same clock, and mark them as controlled by
	 * same policy.
	 */
	for_each_possible_cpu(cpu) {
		if (cpu_clk[cpu] == cpu_clk[policy->cpu]) {
			cpumask_set_cpu(cpu, policy->cpus);
			cpu_work = &per_cpu(cpufreq_work, cpu);
			INIT_WORK(&cpu_work->work, set_cpu_work);
			init_completion(&cpu_work->complete);
		}
	}

	if (cpufreq_frequency_table_cpuinfo(policy, table)) {
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
		policy->cpuinfo.min_freq = CONFIG_MSM_CPU_FREQ_MIN;
		policy->cpuinfo.max_freq = CONFIG_MSM_CPU_FREQ_MAX;
#endif
	}
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
	policy->min = CONFIG_MSM_CPU_FREQ_MIN;
	policy->max = CONFIG_MSM_CPU_FREQ_MAX;
#endif

	cur_freq = clk_get_rate(cpu_clk[policy->cpu])/1000;

	if (cpufreq_frequency_table_target(policy, table, cur_freq,
	    CPUFREQ_RELATION_H, &index) &&
	    cpufreq_frequency_table_target(policy, table, cur_freq,
	    CPUFREQ_RELATION_L, &index)) {
		pr_info("cpufreq: cpu%d at invalid freq: %d\n",
				policy->cpu, cur_freq);
		return -EINVAL;
	}
	/*
	 * Call set_cpu_freq unconditionally so that when cpu is set to
	 * online, frequency limit will always be updated.
	 */
	ret = set_cpu_freq(policy, table[index].frequency,
			   table[index].driver_data);
	if (ret)
		return ret;
	pr_debug("cpufreq: cpu%d init at %d switching to %d\n",
			policy->cpu, cur_freq, table[index].frequency);
	policy->cur = table[index].frequency;
	cpufreq_frequency_table_get_attr(table, policy->cpu);

	return 0;
}

static int msm_cpufreq_cpu_callback(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;
	int rc;

	/* Fail hotplug until this driver can get CPU clocks */
	if (!hotplug_ready)
		return NOTIFY_BAD;

	switch (action & ~CPU_TASKS_FROZEN) {

	case CPU_DYING:
		clk_disable(cpu_clk[cpu]);
		clk_disable(l2_clk);
		break;
	/*
	 * Scale down clock/power of CPU that is dead and scale it back up
	 * before the CPU is brought up.
	 */
	case CPU_DEAD:
		clk_unprepare(cpu_clk[cpu]);
		clk_unprepare(l2_clk);
		break;
	case CPU_UP_CANCELED:
		clk_unprepare(cpu_clk[cpu]);
		clk_unprepare(l2_clk);
		break;
	case CPU_UP_PREPARE:
		rc = clk_prepare(l2_clk);
		if (rc < 0)
			return NOTIFY_BAD;
		rc = clk_prepare(cpu_clk[cpu]);
		if (rc < 0) {
			clk_unprepare(l2_clk);
			return NOTIFY_BAD;
		}
		break;

	case CPU_STARTING:
		rc = clk_enable(l2_clk);
		if (rc < 0)
			return NOTIFY_BAD;
		rc = clk_enable(cpu_clk[cpu]);
		if (rc) {
			clk_disable(l2_clk);
			return NOTIFY_BAD;
		}
		break;

	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block __refdata msm_cpufreq_cpu_notifier = {
	.notifier_call = msm_cpufreq_cpu_callback,
};

static int msm_cpufreq_suspend(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		mutex_lock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
		per_cpu(cpufreq_suspend, cpu).device_suspended = 1;
		mutex_unlock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
	}

	return NOTIFY_DONE;
}

static int msm_cpufreq_resume(void)
{
	int cpu;
#ifndef CONFIG_CPU_BOOST
	int ret;
	struct cpufreq_policy policy;
#endif

	for_each_possible_cpu(cpu) {
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

#ifndef CONFIG_CPU_BOOST
	/*
	 * Freq request might be rejected during suspend, resulting
	 * in policy->cur violating min/max constraint.
	 * Correct the frequency as soon as possible.
	 */
	get_online_cpus();
	for_each_online_cpu(cpu) {
		ret = cpufreq_get_policy(&policy, cpu);
		if (ret)
			continue;
		if (policy.cur <= policy.max && policy.cur >= policy.min)
			continue;
		ret = cpufreq_update_policy(cpu);
		if (ret)
			pr_info("cpufreq: Current frequency violates policy min/max for CPU%d\n",
			       cpu);
		else
			pr_info("cpufreq: Frequency violation fixed for CPU%d\n",
				cpu);
	}
	put_online_cpus();
#endif

	return NOTIFY_DONE;
}

static int msm_cpufreq_pm_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	switch (event) {
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		return msm_cpufreq_resume();
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		return msm_cpufreq_suspend();
	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block msm_cpufreq_pm_notifier = {
	.notifier_call = msm_cpufreq_pm_event,
};

static struct freq_attr *msm_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver msm_cpufreq_driver = {
	/* lps calculations are handled here. */
	.flags		= CPUFREQ_STICKY | CPUFREQ_CONST_LOOPS | CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.init		= msm_cpufreq_init,
	.verify		= msm_cpufreq_verify,
	.target		= msm_cpufreq_target,
	.get		= msm_cpufreq_get_freq,
	.name		= "msm",
	.attr		= msm_freq_attr,
};

#define PROP_PORTS "qcom,cpu-mem-ports"
static struct cpufreq_frequency_table *cpufreq_parse_dt(struct device *dev,
						char *tbl_name, int cpu)
{
	int ret, nf, i, j;
	int num_paths = 0, k;
	u32 *data;
	u32 *ports;
	struct cpufreq_frequency_table *ftbl;
	struct msm_bus_vectors *v = NULL;

	/* Parse optional bus ports parameter */
	if (of_find_property(dev->of_node, PROP_PORTS, &nf)) {
		nf /= sizeof(*ports);
		if (nf % 2)
			return ERR_PTR(-EINVAL);

		ports = devm_kzalloc(dev, nf * sizeof(*ports), GFP_KERNEL);
		if (!ports)
			return ERR_PTR(-ENOMEM);
		ret = of_property_read_u32_array(dev->of_node, PROP_PORTS,
						 ports, nf);

		if (ret)
			return ERR_PTR(ret);
		num_paths = nf / 2;
		nf++;
	}

	/* Parse list of usable CPU frequencies. */
	if (!of_find_property(dev->of_node, tbl_name, &nf))
		return ERR_PTR(-EINVAL);
	nf /= sizeof(*data);

	if (nf == 0)
		return ERR_PTR(-EINVAL);

	data = devm_kzalloc(dev, nf * sizeof(*data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32_array(dev->of_node, tbl_name, data, nf);
	if (ret)
		return ERR_PTR(ret);

	ftbl = devm_kzalloc(dev, (nf + 1) * sizeof(*ftbl), GFP_KERNEL);
	if (!ftbl)
		return ERR_PTR(-ENOMEM);

	if (num_paths) {
		int sz_u = nf * sizeof(*bus_bw.usecase);
		int sz_v = nf * num_paths * sizeof(*bus_vec_lst);
		bus_bw.usecase = devm_kzalloc(dev, sz_u, GFP_KERNEL);
		v = bus_vec_lst = devm_kzalloc(dev, sz_v, GFP_KERNEL);
		if (!bus_bw.usecase || !bus_vec_lst)
			return ERR_PTR(-ENOMEM);
	}

	for (i = 0; i < nf; i++) {
		unsigned long f;

		f = clk_round_rate(cpu_clk[cpu], data[i] * 1000);
		if (IS_ERR_VALUE(f))
			break;
		f /= 1000;

		// override clk_round_rate calculated value for min freq
		if (f < 300000 && f > data[j]) f = data[j];

		/*
		 * Check if this is the last feasible frequency in the table.
		 *
		 * The table listing frequencies higher than what the HW can
		 * support is not an error since the table might be shared
		 * across CPUs in different speed bins. It's also not
		 * sufficient to check if the rounded rate is lower than the
		 * requested rate as it doesn't cover the following example:
		 *
		 * Table lists: 2.2 GHz and 2.5 GHz.
		 * Rounded rate returns: 2.2 GHz and 2.3 GHz.
		 *
		 * In this case, we can CPUfreq to use 2.2 GHz and 2.3 GHz
		 * instead of rejecting the 2.5 GHz table entry.
		 */
		if (i > 0 && f <= ftbl[i-1].frequency)
			break;

		/*
		 * If current frequency being read is greater than the
		 * max frequency allowed skip adding it to the table.
		 */
		if (f > arg_cpu_max_freq) {
			nf = i;
			break;
		}

		ftbl[i].driver_data = i;
		ftbl[i].frequency = f;

		if (num_paths) {
			unsigned int bw_mbps = data[i];
			bus_bw.usecase[i].num_paths = num_paths;
			bus_bw.usecase[i].vectors = v;
			for (k = 0; k < num_paths; k++) {
				v->src = ports[k * 2];
				v->dst = ports[k * 2 + 1];
				v->ib = bw_mbps * 1000000ULL;
				v++;
			}

			pr_info("CPUFREQ: %s: i=%d, bw_mbps=%d\n", __func__, i, bw_mbps);
		}
	}

	bus_bw.num_usecases = i;

	ftbl[i].driver_data = i;
	ftbl[i].frequency = CPUFREQ_TABLE_END;

#ifdef CONFIG_CPU_VOLTAGE_CONTROL
	dts_freq_table =
	devm_kzalloc(dev, (nf + 1) *
		     sizeof(struct cpufreq_frequency_table),
		     GFP_KERNEL);
    
	if (!dts_freq_table)
		return ERR_PTR(-ENOMEM);
    
	for (i = 0, j = 0; i < nf; i++, j += 3)
		dts_freq_table[i].frequency = data[j];

	dts_freq_table[i].frequency = CPUFREQ_TABLE_END;
#endif

	if (ports)
		devm_kfree(dev, ports);

	devm_kfree(dev, data);

	return ftbl;
}

#ifdef CONFIG_CPU_VOLTAGE_CONTROL
bool is_used_by_scaling(unsigned int freq)
{
	unsigned int i, cpu_freq;

	for (i = 0; dts_freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		cpu_freq = dts_freq_table[i].frequency;

	if (cpu_freq == CPUFREQ_ENTRY_INVALID)
		continue;

	if (freq == cpu_freq)
		return true;
	}

	return -EINVAL;
}
#endif

static int __init msm_cpufreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	char clk_name[] = "cpu??_clk";
	char tbl_name[] = "qcom,cpufreq-table-??";
	struct clk *c;
	int cpu;
	struct cpufreq_frequency_table *ftbl;

	l2_clk = devm_clk_get(dev, "l2_clk");
	if (IS_ERR(l2_clk))
		l2_clk = NULL;

	for_each_possible_cpu(cpu) {
		snprintf(clk_name, sizeof(clk_name), "cpu%d_clk", cpu);
		c = devm_clk_get(dev, clk_name);
		if (IS_ERR(c))
			return PTR_ERR(c);
		cpu_clk[cpu] = c;
	}
	hotplug_ready = true;

	/* Use per-policy governor tunable for some targets */
	if (of_property_read_bool(dev->of_node, "qcom,governor-per-policy"))
		msm_cpufreq_driver.flags |= CPUFREQ_HAVE_GOVERNOR_PER_POLICY;

	/* Parse commong cpufreq table for all CPUs */
	ftbl = cpufreq_parse_dt(dev, "qcom,cpufreq-table", 0);
	if (!IS_ERR(ftbl)) {
		for_each_possible_cpu(cpu)
			per_cpu(freq_table, cpu) = ftbl;
		return 0;
	}

	/*
	 * No common table. Parse individual tables for each unique
	 * CPU clock.
	 */
	for_each_possible_cpu(cpu) {
		snprintf(tbl_name, sizeof(tbl_name),
			 "qcom,cpufreq-table-%d", cpu);
		ftbl = cpufreq_parse_dt(dev, tbl_name, cpu);

		/* CPU0 must contain freq table */
		if (cpu == 0 && IS_ERR(ftbl)) {
			dev_err(dev, "Failed to parse CPU0's freq table\n");
			return PTR_ERR(ftbl);
		}
		if (cpu == 0) {
			per_cpu(freq_table, cpu) = ftbl;
			continue;
		}

		if (cpu_clk[cpu] != cpu_clk[cpu - 1] && IS_ERR(ftbl)) {
			dev_err(dev, "Failed to parse CPU%d's freq table\n",
				cpu);
			return PTR_ERR(ftbl);
		}

		/* Use previous CPU's table if it shares same clock */
		if (cpu_clk[cpu] == cpu_clk[cpu - 1]) {
			if (!IS_ERR(ftbl)) {
				dev_warn(dev, "Conflicting tables for CPU%d\n",
					 cpu);
				devm_kfree(dev, ftbl);
			}
			ftbl = per_cpu(freq_table, cpu - 1);
		}
		per_cpu(freq_table, cpu) = ftbl;
	}

	if (bus_bw.usecase) {
		bus_client = msm_bus_scale_register_client(&bus_bw);
		if (!bus_client)
			dev_warn(dev, "Unable to register bus client\n");
	}

	INIT_DELAYED_WORK(&cpufreq_bimc_work, cpufreq_bimc_release);

	return 0;
}

static struct of_device_id match_table[] = {
	{ .compatible = "qcom,msm-cpufreq" },
	{}
};

static struct platform_driver msm_cpufreq_plat_driver = {
	.driver = {
		.name = "msm-cpufreq",
		.of_match_table = match_table,
		.owner = THIS_MODULE,
	},
};

static int __init msm_cpufreq_register(void)
{
	int cpu, rc;

	for_each_possible_cpu(cpu) {
		mutex_init(&(per_cpu(cpufreq_suspend, cpu).suspend_mutex));
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

	rc = platform_driver_probe(&msm_cpufreq_plat_driver,
				   msm_cpufreq_probe);
	if (rc < 0) {
		/* Unblock hotplug if msm-cpufreq probe fails */
		unregister_hotcpu_notifier(&msm_cpufreq_cpu_notifier);
		for_each_possible_cpu(cpu)
			mutex_destroy(&(per_cpu(cpufreq_suspend, cpu).
					suspend_mutex));
		return rc;
	}

	msm_cpufreq_wq = alloc_workqueue("msm-cpufreq", WQ_HIGHPRI, 0);
	register_pm_notifier(&msm_cpufreq_pm_notifier);
	return cpufreq_register_driver(&msm_cpufreq_driver);
}

subsys_initcall(msm_cpufreq_register);

static int __init msm_cpufreq_early_register(void)
{
	return register_hotcpu_notifier(&msm_cpufreq_cpu_notifier);
}
core_initcall(msm_cpufreq_early_register);
