/*
 * Copyright (c) 2015 Linaro Ltd.
 * Author: Pi-Cheng Chen <pi-cheng.chen@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/thermal.h>

// Core voltage and frequency definitions
#define MIN_VOLT_SHIFT           (75000)    // Reduced from 100000 for faster transitions
#define MAX_VOLT_SHIFT           (150000)   // Reduced from 200000
#define MAX_VOLT_LIMIT           (1250000)  // Increased from 1150000 for higher OC potential
#define VOLT_TOL                 (6250)     // Reduced from 10000 for finer control
#define VOLT_STEP                (12500)    // New: 12.5mV stepping for smoother transitions

// Additional timing controls
#define VOLTAGE_SETTLE_TIME      (50)       // Microseconds to wait after voltage change
#define FREQ_SETTLE_TIME         (100)      // Microseconds to wait after frequency change
#define MAX_VOLT_SCALE_TIME      (1000)     // Maximum time for voltage scaling (microseconds)

// Thermal and power limits
#define THERMAL_THROTTLE_TEMP    (85000)    // 85°C throttle point
#define THERMAL_RESET_TEMP       (95000)    // 95°C reset point 
#define MAX_POWER_LIMIT          (4000)     // Maximum power in mW

/*
 * The struct mtk_cpu_dvfs_info holds necessary information for doing CPU DVFS
 * on each CPU power/clock domain of Mediatek SoCs. Each CPU cluster in
 * Mediatek SoCs has two voltage inputs, Vproc and Vsram. In some cases the two
 * voltage inputs need to be controlled under a hardware limitation:
 * 100mV < Vsram - Vproc < 200mV
 *
 * When scaling the clock frequency of a CPU clock domain, the clock source
 * needs to be switched to another stable PLL clock temporarily until
 * the original PLL becomes stable at target frequency.
 */
struct mtk_cpu_dvfs_info {
	struct cpumask cpus;
	struct device *cpu_dev;
	struct regulator *proc_reg;
	struct regulator *sram_reg;
	struct clk *cpu_clk;
	struct clk *inter_clk;
	struct thermal_cooling_device *cdev;
	struct list_head list_head;
	int intermediate_voltage;
	bool need_voltage_tracking;
};

static LIST_HEAD(dvfs_info_list);

static struct mtk_cpu_dvfs_info *mtk_cpu_dvfs_info_lookup(int cpu)
{
	struct mtk_cpu_dvfs_info *info;

	list_for_each_entry(info, &dvfs_info_list, list_head) {
		if (cpumask_test_cpu(cpu, &info->cpus))
			return info;
	}

	return NULL;
}

static int mtk_cpufreq_voltage_tracking(struct mtk_cpu_dvfs_info *info,
                                      int new_vproc)
{
    struct regulator *proc_reg = info->proc_reg;
    struct regulator *sram_reg = info->sram_reg;
    int old_vproc, old_vsram, new_vsram, vsram, vproc, ret;
    int steps, i;
    ktime_t timeout;
    bool limit_reached = false;

    old_vproc = regulator_get_voltage(proc_reg);
    if (old_vproc < 0) {
        pr_err("%s: invalid Vproc value: %d\n", __func__, old_vproc);
        return old_vproc;
    }

    old_vsram = regulator_get_voltage(sram_reg);
    if (old_vsram < 0) {
        pr_err("%s: invalid Vsram value: %d\n", __func__, old_vsram);
        return old_vsram;
    }

    /* Calculate new Vsram target with improved constraints */
    new_vsram = min(new_vproc + MIN_VOLT_SHIFT, MAX_VOLT_LIMIT);

    /* Calculate number of steps for smoother transition */
    steps = (abs(new_vproc - old_vproc) + VOLT_STEP - 1) / VOLT_STEP;
    
    /* Set timeout for voltage scaling */
    timeout = ktime_add_us(ktime_get(), MAX_VOLT_SCALE_TIME);

    if (old_vproc < new_vproc) {
        /* Scaling up voltages */
        for (i = 1; i <= steps && !ktime_after(ktime_get(), timeout); i++) {
            /* Calculate intermediate voltages for this step */
            vproc = min(new_vproc, old_vproc + (i * VOLT_STEP));
            vsram = min(new_vsram, vproc + MIN_VOLT_SHIFT);

            /* Check if we're approaching voltage limits */
            if (vsram + VOLT_TOL >= MAX_VOLT_LIMIT) {
                vsram = MAX_VOLT_LIMIT;
                limit_reached = true;
            }

            /* Set VSRAM first when scaling up */
            if (limit_reached) {
                /* Try exact voltage first, then fallback if needed */
                ret = regulator_set_voltage(sram_reg, vsram, vsram);
                if (ret)
                    ret = regulator_set_voltage(sram_reg, vsram - VOLT_TOL, vsram);
            } else {
                ret = regulator_set_voltage(sram_reg, vsram, vsram + VOLT_TOL);
            }

            if (ret) {
                pr_err("%s: Failed to set Vsram to %d uV: %d\n",
                       __func__, vsram, ret);
                goto volt_restore;
            }

            /* Wait for voltage to settle */
            udelay(VOLTAGE_SETTLE_TIME);

            /* Now set VPROC */
            ret = regulator_set_voltage(proc_reg, vproc, vproc + VOLT_TOL);
            if (ret) {
                pr_err("%s: Failed to set Vproc to %d uV: %d\n",
                       __func__, vproc, ret);
                regulator_set_voltage(sram_reg, old_vsram, old_vsram + VOLT_TOL);
                goto volt_restore;
            }

            /* Wait for voltage to settle */
            udelay(VOLTAGE_SETTLE_TIME);

            /* Verify voltages after setting */
            int curr_vsram = regulator_get_voltage(sram_reg);
            int curr_vproc = regulator_get_voltage(proc_reg);
            
            if (curr_vsram < vsram || curr_vproc < vproc) {
                pr_err("%s: Voltage verification failed. Vsram: %d/%d, Vproc: %d/%d\n",
                       __func__, curr_vsram, vsram, curr_vproc, vproc);
                ret = -EINVAL;
                goto volt_restore;
            }
        }
    } else if (old_vproc > new_vproc) {
        /* Scaling down voltages */
        for (i = 1; i <= steps && !ktime_after(ktime_get(), timeout); i++) {
            /* Calculate intermediate voltages for this step */
            vproc = max(new_vproc, old_vproc - (i * VOLT_STEP));
            vsram = max(new_vsram, vproc + MIN_VOLT_SHIFT);

            /* Set VPROC first when scaling down */
            ret = regulator_set_voltage(proc_reg, vproc, vproc + VOLT_TOL);
            if (ret) {
                pr_err("%s: Failed to set Vproc to %d uV: %d\n",
                       __func__, vproc, ret);
                goto volt_restore;
            }

            udelay(VOLTAGE_SETTLE_TIME);

            /* Then set VSRAM */
            if (vsram + VOLT_TOL >= MAX_VOLT_LIMIT) {
                vsram = MAX_VOLT_LIMIT;
                ret = regulator_set_voltage(sram_reg, vsram, vsram);
                if (ret)
                    ret = regulator_set_voltage(sram_reg, vsram - VOLT_TOL, vsram);
            } else {
                ret = regulator_set_voltage(sram_reg, vsram, vsram + VOLT_TOL);
            }

            if (ret) {
                pr_err("%s: Failed to set Vsram to %d uV: %d\n",
                       __func__, vsram, ret);
                regulator_set_voltage(proc_reg, old_vproc, old_vproc + VOLT_TOL);
                goto volt_restore;
            }

            udelay(VOLTAGE_SETTLE_TIME);

            /* Verify voltages */
            int curr_vsram = regulator_get_voltage(sram_reg);
            int curr_vproc = regulator_get_voltage(proc_reg);
            
            if (curr_vsram > old_vsram || curr_vproc > old_vproc) {
                pr_err("%s: Voltage verification failed during scaling down\n",
                       __func__);
                ret = -EINVAL;
                goto volt_restore;
            }
        }
    }

    /* Check if we hit the timeout */
    if (ktime_after(ktime_get(), timeout)) {
        pr_warn("%s: Voltage scaling timeout reached\n", __func__);
        ret = -ETIMEDOUT;
        goto volt_restore;
    }

    return 0;

volt_restore:
    /* Restore original voltages on failure */
    regulator_set_voltage(proc_reg, old_vproc, old_vproc + VOLT_TOL);
    regulator_set_voltage(sram_reg, old_vsram, old_vsram + VOLT_TOL);
    return ret;
}

static int mtk_cpufreq_set_voltage(struct mtk_cpu_dvfs_info *info, int vproc)
{
	if (info->need_voltage_tracking)
		return mtk_cpufreq_voltage_tracking(info, vproc);
	else
		return regulator_set_voltage(info->proc_reg, vproc,
					     vproc + VOLT_TOL);
}

// Enhanced frequency transition function
static int mtk_cpufreq_set_target(struct cpufreq_policy *policy,
                                 unsigned int index)
{
    struct cpufreq_frequency_table *freq_table = policy->freq_table;
    struct clk *cpu_clk = policy->clk;
    struct clk *armpll = clk_get_parent(cpu_clk);
    struct mtk_cpu_dvfs_info *info = policy->driver_data;
    struct thermal_zone_device *tz;
    int temp, vproc, old_vproc, inter_vproc, target_vproc, ret;
    unsigned long freq_hz, old_freq_hz;
    ktime_t timeout;

    // Get current temperature
    tz = thermal_zone_get_zone_by_name("cpu-thermal");
    if (!IS_ERR(tz)) {
        ret = thermal_zone_get_temp(tz, &temp);
        if (!ret && temp >= THERMAL_THROTTLE_TEMP) {
            if (temp >= THERMAL_RESET_TEMP)
                return -EINVAL;
            // Adjust index for thermal throttling
            index = cpufreq_cooling_get_level(policy->cpu, temp);
        }
    }

    inter_vproc = info->intermediate_voltage;
    old_freq_hz = clk_get_rate(cpu_clk);
    freq_hz = freq_table[index].frequency * 1000;

    // Set timeout for frequency transition
    timeout = ktime_add_us(ktime_get(), FREQ_SETTLE_TIME);

    // Voltage preparation phase
    if (freq_hz > old_freq_hz) {
        // Need higher voltage for higher frequency
        target_vproc = inter_vproc > vproc ? inter_vproc : vproc;
        ret = mtk_cpufreq_set_voltage(info, target_vproc);
        if (ret) {
            pr_err("CPU voltage scale up failed: %d\n", ret);
            return ret;
        }
        udelay(VOLTAGE_SETTLE_TIME);
    }

    // Frequency transition phase
    ret = clk_set_parent(cpu_clk, info->inter_clk);
    if (ret)
        goto freq_fail;

    ret = clk_set_rate_timeout(armpll, freq_hz, timeout);
    if (ret)
        goto freq_fail;

    ret = clk_set_parent(cpu_clk, armpll);
    if (ret)
        goto freq_fail;

    // Post frequency adjustment voltage phase
    if (freq_hz < old_freq_hz) {
        // Can reduce voltage after lowering frequency
        ret = mtk_cpufreq_set_voltage(info, vproc);
        if (ret) {
            pr_err("CPU voltage scale down failed: %d\n", ret);
            goto freq_fail;
        }
    }

    return 0;

freq_fail:
    pr_err("CPU frequency transition failed: %d\n", ret);
    // Attempt recovery
    clk_set_parent(cpu_clk, info->inter_clk);
    clk_set_rate(armpll, old_freq_hz);
    clk_set_parent(cpu_clk, armpll);
    return ret;
}

// Enhanced CPU cooling implementation
static struct thermal_cooling_device_ops mtk_cpu_cooling_ops = {
    .get_max_state = cpu_cooling_get_max_state,
    .get_cur_state = cpu_cooling_get_cur_state,
    .set_cur_state = cpu_cooling_set_cur_state,
    .get_requested_power = cpu_cooling_get_requested_power,
    .state2power = cpu_cooling_state2power,
    .power2state = cpu_cooling_power2state,
};

// Enhanced CPU initialization
static int mtk_cpufreq_init(struct cpufreq_policy *policy)
{
    struct mtk_cpu_dvfs_info *info;
    struct cpufreq_frequency_table *freq_table;
    struct dev_pm_opp *opp;
    unsigned long rate;
    int ret;

    info = mtk_cpu_dvfs_info_lookup(policy->cpu);
    if (!info)
        return -EINVAL;

    ret = dev_pm_opp_init_cpufreq_table(info->cpu_dev, &freq_table);
    if (ret)
        return ret;

    // Optimize OPP table
    for (rate = 0; ; rate++) {
        opp = dev_pm_opp_find_freq_ceil(info->cpu_dev, &rate);
        if (IS_ERR(opp)) {
            if (PTR_ERR(opp) == -ERANGE)
                break;
            ret = PTR_ERR(opp);
            goto free_table;
        }

        // Apply custom voltage optimization
        unsigned long voltage = dev_pm_opp_get_voltage(opp);
        // Optimize voltage based on frequency
        if (rate <= 1000000000)  // Below 1GHz
            voltage = voltage - (voltage * 5 / 100);  // Reduce by 5%
        else if (rate <= 2000000000)  // 1-2GHz
            voltage = voltage - (voltage * 3 / 100);  // Reduce by 3%
        // Above 2GHz keeps original voltage for stability

        dev_pm_opp_put(opp);

        ret = dev_pm_opp_adjust_voltage(info->cpu_dev, rate, voltage);
        if (ret)
            goto free_table;
    }

    policy->freq_table = freq_table;
    policy->driver_data = info;
    policy->clk = info->cpu_clk;
    
    // Set conservative default governor
    policy->governor = cpufreq_default_governor();
    
    // Enable fast frequency switching if supported
    policy->fast_switch_possible = true;

    return 0;

free_table:
    dev_pm_opp_free_cpufreq_table(info->cpu_dev, &freq_table);
    return ret;
}

#define DYNAMIC_POWER "dynamic-power-coefficient"

static void mtk_cpufreq_ready(struct cpufreq_policy *policy)
{
	struct mtk_cpu_dvfs_info *info = policy->driver_data;

	info->cdev = of_cpufreq_cooling_register(policy);
}

static int mtk_cpu_dvfs_info_init(struct mtk_cpu_dvfs_info *info, int cpu)
{
	struct device *cpu_dev;
	struct regulator *proc_reg = ERR_PTR(-ENODEV);
	struct regulator *sram_reg = ERR_PTR(-ENODEV);
	struct clk *cpu_clk = ERR_PTR(-ENODEV);
	struct clk *inter_clk = ERR_PTR(-ENODEV);
	struct dev_pm_opp *opp;
	unsigned long rate;
	int ret;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		pr_err("failed to get cpu%d device\n", cpu);
		return -ENODEV;
	}

	cpu_clk = clk_get(cpu_dev, "cpu");
	if (IS_ERR(cpu_clk)) {
		if (PTR_ERR(cpu_clk) == -EPROBE_DEFER)
			pr_warn("cpu clk for cpu%d not ready, retry.\n", cpu);
		else
			pr_err("failed to get cpu clk for cpu%d\n", cpu);

		ret = PTR_ERR(cpu_clk);
		return ret;
	}

	inter_clk = clk_get(cpu_dev, "intermediate");
	if (IS_ERR(inter_clk)) {
		if (PTR_ERR(inter_clk) == -EPROBE_DEFER)
			pr_warn("intermediate clk for cpu%d not ready, retry.\n",
				cpu);
		else
			pr_err("failed to get intermediate clk for cpu%d\n",
			       cpu);

		ret = PTR_ERR(inter_clk);
		goto out_free_resources;
	}

	proc_reg = regulator_get_exclusive(cpu_dev, "proc");
	if (IS_ERR(proc_reg)) {
		if (PTR_ERR(proc_reg) == -EPROBE_DEFER)
			pr_warn("proc regulator for cpu%d not ready, retry.\n",
				cpu);
		else
			pr_err("failed to get proc regulator for cpu%d\n",
			       cpu);

		ret = PTR_ERR(proc_reg);
		goto out_free_resources;
	}

	/* Both presence and absence of sram regulator are valid cases. */
	sram_reg = regulator_get_exclusive(cpu_dev, "sram");

	/* Get OPP-sharing information from "operating-points-v2" bindings */
	ret = dev_pm_opp_of_get_sharing_cpus(cpu_dev, &info->cpus);
	if (ret) {
		pr_err("failed to get OPP-sharing information for cpu%d\n",
		       cpu);
		goto out_free_resources;
	}

	ret = dev_pm_opp_of_cpumask_add_table(&info->cpus);
	if (ret) {
		pr_warn("no OPP table for cpu%d\n", cpu);
		goto out_free_resources;
	}

	/* Search a safe voltage for intermediate frequency. */
	rate = clk_get_rate(inter_clk);
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
	if (IS_ERR(opp)) {
		pr_err("failed to get intermediate opp for cpu%d\n", cpu);
		ret = PTR_ERR(opp);
		goto out_free_opp_table;
	}
	info->intermediate_voltage = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	info->cpu_dev = cpu_dev;
	info->proc_reg = proc_reg;
	info->sram_reg = IS_ERR(sram_reg) ? NULL : sram_reg;
	info->cpu_clk = cpu_clk;
	info->inter_clk = inter_clk;

	/*
	 * If SRAM regulator is present, software "voltage tracking" is needed
	 * for this CPU power domain.
	 */
	info->need_voltage_tracking = !IS_ERR(sram_reg);

	return 0;

out_free_opp_table:
	dev_pm_opp_of_cpumask_remove_table(&info->cpus);

out_free_resources:
	if (!IS_ERR(proc_reg))
		regulator_put(proc_reg);
	if (!IS_ERR(sram_reg))
		regulator_put(sram_reg);
	if (!IS_ERR(cpu_clk))
		clk_put(cpu_clk);
	if (!IS_ERR(inter_clk))
		clk_put(inter_clk);

	return ret;
}

static void mtk_cpu_dvfs_info_release(struct mtk_cpu_dvfs_info *info)
{
	if (!IS_ERR(info->proc_reg))
		regulator_put(info->proc_reg);
	if (!IS_ERR(info->sram_reg))
		regulator_put(info->sram_reg);
	if (!IS_ERR(info->cpu_clk))
		clk_put(info->cpu_clk);
	if (!IS_ERR(info->inter_clk))
		clk_put(info->inter_clk);

	dev_pm_opp_of_cpumask_remove_table(&info->cpus);
}

static int mtk_cpufreq_init(struct cpufreq_policy *policy)
{
	struct mtk_cpu_dvfs_info *info;
	struct cpufreq_frequency_table *freq_table;
	int ret;

	info = mtk_cpu_dvfs_info_lookup(policy->cpu);
	if (!info) {
		pr_err("dvfs info for cpu%d is not initialized.\n",
		       policy->cpu);
		return -EINVAL;
	}

	ret = dev_pm_opp_init_cpufreq_table(info->cpu_dev, &freq_table);
	if (ret) {
		pr_err("failed to init cpufreq table for cpu%d: %d\n",
		       policy->cpu, ret);
		return ret;
	}

	cpumask_copy(policy->cpus, &info->cpus);
	policy->freq_table = freq_table;
	policy->driver_data = info;
	policy->clk = info->cpu_clk;

	return 0;
}

static int mtk_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct mtk_cpu_dvfs_info *info = policy->driver_data;

	cpufreq_cooling_unregister(info->cdev);
	dev_pm_opp_free_cpufreq_table(info->cpu_dev, &policy->freq_table);

	return 0;
}

static struct cpufreq_driver mtk_cpufreq_driver = {
	.flags = CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_HAVE_GOVERNOR_PER_POLICY,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = mtk_cpufreq_set_target,
	.get = cpufreq_generic_get,
	.init = mtk_cpufreq_init,
	.exit = mtk_cpufreq_exit,
	.ready = mtk_cpufreq_ready,
	.name = "mtk-cpufreq",
	.attr = cpufreq_generic_attr,
};

static int mtk_cpufreq_probe(struct platform_device *pdev)
{
	struct mtk_cpu_dvfs_info *info, *tmp;
	int cpu, ret;

	for_each_possible_cpu(cpu) {
		info = mtk_cpu_dvfs_info_lookup(cpu);
		if (info)
			continue;

		info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
		if (!info) {
			ret = -ENOMEM;
			goto release_dvfs_info_list;
		}

		ret = mtk_cpu_dvfs_info_init(info, cpu);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to initialize dvfs info for cpu%d\n",
				cpu);
			goto release_dvfs_info_list;
		}

		list_add(&info->list_head, &dvfs_info_list);
	}

	ret = cpufreq_register_driver(&mtk_cpufreq_driver);
	if (ret) {
		dev_err(&pdev->dev, "failed to register mtk cpufreq driver\n");
		goto release_dvfs_info_list;
	}

	return 0;

release_dvfs_info_list:
	list_for_each_entry_safe(info, tmp, &dvfs_info_list, list_head) {
		mtk_cpu_dvfs_info_release(info);
		list_del(&info->list_head);
	}

	return ret;
}

static struct platform_driver mtk_cpufreq_platdrv = {
	.driver = {
		.name	= "mtk-cpufreq",
	},
	.probe		= mtk_cpufreq_probe,
};

/* List of machines supported by this driver */
static const struct of_device_id mtk_cpufreq_machines[] __initconst = {
	{ .compatible = "mediatek,mt2701", },
	{ .compatible = "mediatek,mt2712", },
	{ .compatible = "mediatek,mt7622", },
	{ .compatible = "mediatek,mt7623", },
	{ .compatible = "mediatek,mt817x", },
	{ .compatible = "mediatek,mt8173", },
	{ .compatible = "mediatek,mt8176", },

	{ }
};
MODULE_DEVICE_TABLE(of, mtk_cpufreq_machines);

static int __init mtk_cpufreq_driver_init(void)
{
	struct device_node *np;
	const struct of_device_id *match;
	struct platform_device *pdev;
	int err;

	np = of_find_node_by_path("/");
	if (!np)
		return -ENODEV;

	match = of_match_node(mtk_cpufreq_machines, np);
	of_node_put(np);
	if (!match) {
		pr_debug("Machine is not compatible with mtk-cpufreq\n");
		return -ENODEV;
	}

	err = platform_driver_register(&mtk_cpufreq_platdrv);
	if (err)
		return err;

	/*
	 * Since there's no place to hold device registration code and no
	 * device tree based way to match cpufreq driver yet, both the driver
	 * and the device registration codes are put here to handle defer
	 * probing.
	 */
	pdev = platform_device_register_simple("mtk-cpufreq", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("failed to register mtk-cpufreq platform device\n");
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(mtk_cpufreq_driver_init);

MODULE_DESCRIPTION("MediaTek CPUFreq driver");
MODULE_AUTHOR("Pi-Cheng Chen <pi-cheng.chen@linaro.org>");
MODULE_LICENSE("GPL v2");
