// SPDX-License-Identifier: GPL-2.0
// ExoticBalance Hybrid - Property of Morat Engine 

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/string.h>
#include <linux/cpuidle.h>

#define LIGHT_INTERVAL_MS 5000
#define HEAVY_INTERVAL_MS 30000
#define MIN_DELTA_IRQS_BASE 800
#define MAX_CPU_TEMP_THRESHOLD 70
#define MAX_IRQ_MIGRATE_PER_TICK 5 

extern unsigned int kstat_irqs_cpu(unsigned int irq, int cpu);

static bool exoticbalance_enabled = true;
module_param(exoticbalance_enabled, bool, 0644);
MODULE_PARM_DESC(exoticbalance_enabled, "Enable ExoticBalance");

static struct delayed_work balance_work;
static unsigned int *cpu_irq_count;
static unsigned int *cpu_irq_last;
static bool heavy_tick = false;

#if defined(cpuidle_cpu_in_idle)
static inline bool cpu_is_idle_wrap(int cpu)
{
    return cpuidle_cpu_in_idle(cpu);
}
#else
static inline bool cpu_is_idle_wrap(int cpu)
{
    return false;
}
#endif

/* Blacklist IRQ */
static const char *const irq_name_blacklist[] = {
    "mdss","sde","dsi","kgsl","adreno","msm_gpu",
    "input","touch","synaptics","fts","goodix",
    "ufs","ufshcd","qcom-ufshcd","sdc",
    "wlan","wifi","rmnet","ipa","qcom,sps","bam","modem","qrtr",
    "pmic","smb","bms","timer","hrtimer","watchdog","thermal","cpu",NULL
};

static inline bool is_irq_blacklisted(int irq)
{
    struct irq_desc *desc = irq_to_desc(irq);
    const char *name;
    int i;

    if (!desc || !desc->action || !desc->action->name)
        return false;

    name = desc->action->name;
    for (i = 0; irq_name_blacklist[i]; i++) {
        if (strlen(name) >= strlen(irq_name_blacklist[i]) &&
            strnstr(name, irq_name_blacklist[i], strlen(name)))
            return true;
    }
    return false;
}

static int get_max_cpu_temp(void)
{
#if IS_ENABLED(CONFIG_THERMAL)
    struct thermal_zone_device *tz;
    int temp = 0;
    tz = thermal_zone_get_zone_by_name("cpu-thermal");
    if (!IS_ERR(tz))
        tz->ops->get_temp(tz, &temp);
    return temp / 1000;
#else
    return 0;
#endif
}

static unsigned int get_cpu_max_freq(int cpu)
{
    struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);
    unsigned int freq = 0;
    if (policy) {
        freq = policy->cpuinfo.max_freq;
        cpufreq_cpu_put(policy);
    }
    return freq;
}

static bool is_cpu_big(int cpu)
{
    return get_cpu_max_freq(cpu) >= 2000000;
}

static void migrate_irqs_limited(int from, int to, int *migrated)
{
    int irq;
    cpumask_t new_mask;

    for (irq = 0; irq < nr_irqs; irq++) {
        if (*migrated >= MAX_IRQ_MIGRATE_PER_TICK)
            break;

        if (!irq_can_set_affinity(irq))
            continue;

        if (is_irq_blacklisted(irq))
            continue;

        if (cpu_is_idle_wrap(to))
            continue;

        cpumask_clear(&new_mask);
        cpumask_set_cpu(to, &new_mask);
        irq_set_affinity(irq, &new_mask);
        (*migrated)++;
    }
}

static void exotic_balance_irq(struct work_struct *work)
{
    int irq, cpu;
    unsigned int max_irq = 0, min_irq = UINT_MAX;
    int max_cpu = -1, min_cpu = -1;
    int delta_sum = 0, delta_avg = 0;
    int dynamic_threshold;
    int migrated = 0;

    if (!exoticbalance_enabled)
        goto schedule_next;

    memset(cpu_irq_count, 0, sizeof(unsigned int) * nr_cpu_ids);

    for (irq = 0; irq < nr_irqs; irq++) {
        if (!heavy_tick && is_irq_blacklisted(irq))
            continue;

        for_each_online_cpu(cpu)
            cpu_irq_count[cpu] += kstat_irqs_cpu(irq, cpu);
    }

    for_each_online_cpu(cpu) {
        unsigned int delta = cpu_irq_count[cpu] - cpu_irq_last[cpu];
        delta_sum += delta;

        if (delta > max_irq) { max_irq = delta; max_cpu = cpu; }
        if (delta < min_irq) { min_irq = delta; min_cpu = cpu; }

        cpu_irq_last[cpu] = cpu_irq_count[cpu];
    }

    delta_avg = delta_sum / num_online_cpus();
    dynamic_threshold = delta_avg + MIN_DELTA_IRQS_BASE;

    if (max_cpu >= 0 && min_cpu >= 0 && max_cpu != min_cpu) {
        if ((max_irq - min_irq) >= dynamic_threshold &&
            !is_cpu_big(min_cpu) && is_cpu_big(max_cpu) &&
            get_max_cpu_temp() < MAX_CPU_TEMP_THRESHOLD) {
            migrate_irqs_limited(max_cpu, min_cpu, &migrated);
        }
    }

    heavy_tick = !heavy_tick; 
schedule_next:
    if (cpu_is_idle_wrap(max_cpu))
        schedule_delayed_work(&balance_work, msecs_to_jiffies(HEAVY_INTERVAL_MS));
    else
        schedule_delayed_work(&balance_work, msecs_to_jiffies(LIGHT_INTERVAL_MS));
}

static int __init exoticbalance_init(void)
{
    cpu_irq_count = kzalloc(sizeof(unsigned int) * nr_cpu_ids, GFP_KERNEL);
    cpu_irq_last = kzalloc(sizeof(unsigned int) * nr_cpu_ids, GFP_KERNEL);
    if (!cpu_irq_count || !cpu_irq_last)
        return -ENOMEM;

    INIT_DELAYED_WORK(&balance_work, exotic_balance_irq);
    schedule_delayed_work(&balance_work, msecs_to_jiffies(LIGHT_INTERVAL_MS));
    pr_info("ExoticBalance Hybrid: Initialized\n");
    return 0;
}

static void __exit exoticbalance_exit(void)
{
    cancel_delayed_work_sync(&balance_work);
    kfree(cpu_irq_count);
    kfree(cpu_irq_last);
    pr_info("ExoticBalance Hybrid: Unloaded\n");
}

module_init(exoticbalance_init);
module_exit(exoticbalance_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tobrut Exotic");
MODULE_DESCRIPTION("ExoticBalance Hybrid: friendly IRQ balancer");