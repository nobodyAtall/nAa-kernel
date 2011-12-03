/*
 * Author: nobodyAtall <nobodyAtall at xda-developers>
 * Code adopted from doixanh's <doixanh at xda-developers> x8oc module
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>	
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/jiffies.h>
#include <linux/cpufreq.h>
#include <mach/msm_iomap.h>
#include <../clock.h>
#include <../acpuclock.h>
#include <asm/mach-types.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <linux/kallsyms.h>
#include <asm/cputime.h>

// defines
#define MODULE_NAME 			"x10minioc"
#define PROCFS_NAME 			"x10minioc"
#define DXDBG(x)					

#define DEVICE_NAME				"X10mini"
//#define OFS_KALLSYMS_LOOKUP_NAME	0xC00AF6D8			// kallsyms_lookup_name for stock
#define OFS_KALLSYMS_LOOKUP_NAME	0xc00ac718			// kallsyms_lookup_name for nAa-06 x10mini

// struct definition
struct clkctl_acpu_speed_dx {
	unsigned int	use_for_scaling;
	unsigned int	a11clk_khz;
	int		pll;
	unsigned int	a11clk_src_sel;
	unsigned int	a11clk_src_div;
	unsigned int	ahbclk_khz;
	unsigned int	ahbclk_div;
	int		vdd;
	unsigned int 	axiclk_khz;
	unsigned long	lpj; /* loops_per_jiffy */
/* Pointers in acpu_freq_tbl[] for max up/down steppings. */
	struct clkctl_acpu_speed_dx *down[3];
	struct clkctl_acpu_speed_dx *up[3];
};


struct HexMapping {
	char chr;
	int value;
};

static struct HexMapping HexMap[22] = {
	{'0', 0}, {'1', 1},
	{'2', 2}, {'3', 3},
	{'4', 4}, {'5', 5},
	{'6', 6}, {'7', 7},
	{'8', 8}, {'9', 9},
	{'A', 10}, {'B', 11},
	{'C', 12}, {'D', 13},
	{'E', 14}, {'F', 15},
	{'a', 10}, {'b', 11},
	{'c', 12}, {'d', 13},
	{'e', 14}, {'f', 15}};

struct clock_state {
	struct clkctl_acpu_speed_dx	*current_speed;
	struct mutex		lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			max_speed_delta_khz;
	uint32_t			vdd_switch_time_us;
	unsigned long		max_axi_khz;
};

struct cpufreq_stats {
	unsigned int cpu;
	unsigned int total_trans;
	unsigned long long  last_time;
	unsigned int max_state;
	unsigned int state_num;
	unsigned int last_index;
	cputime64_t *time_in_state;
	unsigned int *freq_table;
	unsigned int *trans_table;
};

// enum types
enum {
	ACPU_PLL_TCXO	= -1,
	ACPU_PLL_0	= 0,
	ACPU_PLL_1,
	ACPU_PLL_2,
	ACPU_PLL_3,
	ACPU_PLL_END,
};
	

// variables
static int x8_oc_ratio;
static struct proc_dir_entry *x10minioc_proc;
static unsigned int dump_addr;
static unsigned int dump_size;
static bool raw_dump;
typedef void (*acpuclk_set_div_org_func_type) (const struct clkctl_acpu_speed_dx*);
static struct clkctl_acpu_speed_dx *standard_clocks;
static int num_freq_added;
static struct cpufreq_frequency_table *freq_table_dx;
static struct clkctl_acpu_speed_dx *acpu_freq_tbl_dx;

// external variables / functions
typedef unsigned long (*kallsyms_lookup_name_type)(const char *name);
static kallsyms_lookup_name_type kallsyms_lookup_name_dx;
static struct clock_state *drv_state_dx;
static struct clkctl_acpu_speed_dx *acpu_freq_tbl_org;
static struct cpufreq_stats *cpufreq_stat_dx;

static unsigned int ofs_cpufreq_cpu_data;
typedef int (*ebi1_clk_set_min_rate_type)(enum clkvote_client client, unsigned long rate);
static ebi1_clk_set_min_rate_type ebi1_clk_set_min_rate_dx;
typedef int (*acpuclk_set_vdd_level_type)(int vdd);
static acpuclk_set_vdd_level_type acpuclk_set_vdd_level_dx;
typedef int (*pc_pll_request_type)(unsigned id, unsigned on);
static pc_pll_request_type pc_pll_request_dx; 


// hex to int
int _httoi(const char *value) {
	char *s;
	int result;
	int i;
	bool firsttime;

	result = 0;
	memcpy(&s, &value, 4);
	firsttime = true;
	while (*s != '\0') {
		bool found = false;
		for (i = 0; i < 22; i++) {
			if (*s == HexMap[i].chr) {
				if (!firsttime) result <<= 4;
				result |= HexMap[i].value;
				found = true;
				break;
			}
		}
		if (!found) break;
		s++;
		firsttime = false;
  	}
	return result;
}

// inline memory patch an unsigned integer
static void patch(unsigned int addr, unsigned int value) {
	*(unsigned int*)addr = value;
}

// patch to an jump obcode
static void patch_to_jmp(unsigned int addr, void * func) {
	int write_value;
	// calculate the offset
	write_value = ((((unsigned int)func - 8 - addr) >> 2) & 0x00FFFFFF);
	// add the unconditional jump opcode
	write_value |= 0xEA000000;
	// and patch it
	patch(addr, write_value);
}

// this is the function that should be replaced with standard acpuclk_set_div
// so it could apply the correct pll and divisor
static void acpuclk_set_div_dx(const struct clkctl_acpu_speed_dx *hunt_s) {
	uint32_t reg_clkctl, reg_clksel, clk_div, src_sel, a11_div;

	reg_clksel = readl(A11S_CLK_SEL_ADDR);

	/* AHB_CLK_DIV */
	clk_div = (reg_clksel >> 1) & 0x03;
	a11_div=hunt_s->a11clk_src_div;
	if(hunt_s->a11clk_khz > 600000) {
		a11_div=0;
		DXDBG(printk(KERN_INFO "%s: setting ratio %2X\n", __FUNCTION__, hunt_s->a11clk_khz / 19200);)
		writel(hunt_s->a11clk_khz / 19200, MSM_CLK_CTL_BASE+0x33C);		// specified ratio
		udelay(50);
	}

	/* CLK_SEL_SRC1NO */
	src_sel = reg_clksel & 1;

	/*
	 * If the new clock divider is higher than the previous, then
	 * program the divider before switching the clock
	 */
	if (hunt_s->ahbclk_div > clk_div) {
		reg_clksel &= ~(0x3 << 1);
		reg_clksel |= (hunt_s->ahbclk_div << 1);
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}

	/* Program clock source and divider */
	reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
	reg_clkctl &= ~(0xFF << (8 * src_sel));
	reg_clkctl |= hunt_s->a11clk_src_sel << (4 + 8 * src_sel);
	reg_clkctl |= a11_div << (0 + 8 * src_sel);
	writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

	/* Program clock source selection */
	reg_clksel ^= 1;
	writel(reg_clksel, A11S_CLK_SEL_ADDR);

	/*
	 * If the new clock divider is lower than the previous, then
	 * program the divider after switching the clock
	 */
	if (hunt_s->ahbclk_div < clk_div) {
		reg_clksel &= ~(0x3 << 1);
		reg_clksel |= (hunt_s->ahbclk_div << 1);
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}
}



// hijack calls to set_rate, just to call our set div.
int acpuclk_set_rate_dx(unsigned long rate, enum setrate_reason reason)
{
	uint32_t reg_clkctl;
	struct clkctl_acpu_speed_dx *cur_s, *tgt_s, *strt_s;
	int res, rc = 0;
	unsigned int plls_enabled = 0, pll;

	if (reason == SETRATE_CPUFREQ)
		mutex_lock(&drv_state_dx->lock);

	// DXDBG(printk(KERN_INFO "%s: set rate %lu\n", __FUNCTION__, rate);)
	
	cur_s = drv_state_dx->current_speed;
	strt_s = cur_s;

	WARN_ONCE(cur_s == NULL, "acpuclk_set_rate: not initialized\n");
	if (cur_s == NULL) {
		rc = -ENOENT;
		// DXDBG(printk(KERN_INFO "%s: cur_s null \n", __FUNCTION__);)
		goto out;
	}

	if (rate == cur_s->a11clk_khz) {
		// DXDBG(printk(KERN_INFO "%s: rate=cur_s->a11clk_khz \n", __FUNCTION__);)
		goto out;
	}

	//DXDBG(printk(KERN_INFO "%s: looking for target\n", __FUNCTION__);)
	for (tgt_s = acpu_freq_tbl_dx; tgt_s->a11clk_khz != 0; tgt_s++) {
		//DXDBG(printk(KERN_INFO "%s: tgt_s %08X a11clk_khz = %d\n", __FUNCTION__, (unsigned int) tgt_s, tgt_s->a11clk_khz );)
		if (tgt_s->a11clk_khz == rate)
			break;
	}

	if (tgt_s->a11clk_khz == 0) {
		rc = -EINVAL;
		// DXDBG(printk(KERN_INFO "%s: tgt_s->a11clk_khz == 0 \n", __FUNCTION__);)
		goto out;
	}

	// DXDBG(printk(KERN_INFO "%s: choosing\n", __FUNCTION__);)
	/* Choose the highest speed at or below 'rate' with same PLL. */
	if (reason != SETRATE_CPUFREQ
	    && tgt_s->a11clk_khz < cur_s->a11clk_khz) {
		while (tgt_s->pll != ACPU_PLL_TCXO && tgt_s->pll != cur_s->pll)
			tgt_s--;
	}
	
	// DXDBG(printk(KERN_INFO "%s: comparing pll & tcx0\n", __FUNCTION__);)
	if (strt_s->pll != ACPU_PLL_TCXO)
		plls_enabled |= 1 << strt_s->pll;

	// DXDBG(printk(KERN_INFO "%s: comparing reason & setrate_cpufreq\n", __FUNCTION__);)
	if (reason == SETRATE_CPUFREQ) {
		if (strt_s->pll != tgt_s->pll && tgt_s->pll != ACPU_PLL_TCXO) {
			rc = pc_pll_request_dx(tgt_s->pll, 1);
			if (rc < 0) {
				pr_err("PLL%d enable failed (%d)\n",
					tgt_s->pll, rc);
				goto out;
			}
			plls_enabled |= 1 << tgt_s->pll;
		}
	}
	/* Need to do this when coming out of power collapse since some modem
	 * firmwares reset the VDD when the application processor enters power
	 * collapse. */
	if (reason == SETRATE_CPUFREQ || reason == SETRATE_PC) {
		/* Increase VDD if needed. */
		if (tgt_s->vdd > cur_s->vdd) {
			DXDBG(printk(KERN_INFO "%s: increasing to new vdd %d\n", __FUNCTION__, tgt_s->vdd);)
			rc = acpuclk_set_vdd_level_dx(tgt_s->vdd);
			if (rc < 0) {
				pr_err("Unable to switch ACPU vdd (%d)\n", rc);
				goto out;
			}
		}
	}

	// DXDBG(printk(KERN_INFO "%s: set reg clk\n", __FUNCTION__);)
	/* Set wait states for CPU inbetween frequency changes */
	reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
	reg_clkctl |= (100 << 16); /* set WT_ST_CNT */
	writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

	/*dprintk("Switching from ACPU rate %u KHz -> %u KHz\n",
		       strt_s->a11clk_khz, tgt_s->a11clk_khz);*/

	DXDBG(printk(KERN_INFO "%s Switching from ACPU rate %u KHz -> %u KHz\n", __FUNCTION__,
		       strt_s->a11clk_khz, tgt_s->a11clk_khz);)
	while (cur_s != tgt_s) {
		/*
		 * Always jump to target freq if within 256mhz, regulardless of
		 * PLL. If differnece is greater, use the predefinied
		 * steppings in the table.
		 */
		int d = abs((int)(cur_s->a11clk_khz - tgt_s->a11clk_khz));
		DXDBG(printk(KERN_INFO "%s: d %d\n", __FUNCTION__, d);)
		if (d > drv_state_dx->max_speed_delta_khz) {

			if (tgt_s->a11clk_khz > cur_s->a11clk_khz) {
				/* Step up: jump to target PLL as early as
				 * possible so indexing using TCXO (up[-1])
				 * never occurs. */
				if (likely(cur_s->up[tgt_s->pll]))
					cur_s = cur_s->up[tgt_s->pll];
				else
					cur_s = cur_s->up[cur_s->pll];
			} else {
				/* Step down: stay on current PLL as long as
				 * possible so indexing using TCXO (down[-1])
				 * never occurs. */
				if (likely(cur_s->down[cur_s->pll]))
					cur_s = cur_s->down[cur_s->pll];
				else
					cur_s = cur_s->down[tgt_s->pll];
			}

			if (cur_s == NULL) { /* This should not happen. */
				pr_err("No stepping frequencies found. "
					"strt_s:%u tgt_s:%u\n",
					strt_s->a11clk_khz, tgt_s->a11clk_khz);
				rc = -EINVAL;
				goto out;
			}

		} else {
			cur_s = tgt_s;
		}

		// DXDBG(printk(KERN_INFO "STEP khz = %u, pll = %d\n",	cur_s->a11clk_khz, cur_s->pll);)

		if (cur_s->pll != ACPU_PLL_TCXO
		    && !(plls_enabled & (1 << cur_s->pll))) {
			rc = pc_pll_request_dx(cur_s->pll, 1);
			if (rc < 0) {
				pr_err("PLL%d enable failed (%d)\n",
					cur_s->pll, rc);
				goto out;
			}
			plls_enabled |= 1 << cur_s->pll;
		}

		// DXDBG(printk(KERN_INFO "%s: setting div, cur_s %8X\n", __FUNCTION__, (unsigned int)cur_s);)
		acpuclk_set_div_dx(cur_s);
		drv_state_dx->current_speed = cur_s;
		/* Re-adjust lpj for the new clock speed. */
		loops_per_jiffy = cur_s->lpj;
		// DXDBG(printk(KERN_INFO "%s: delay\n", __FUNCTION__);)
		udelay(drv_state_dx->acpu_switch_time_us);
	}

	/* Nothing else to do for SWFI. */
	if (reason == SETRATE_SWFI)
		goto out;

	/* Change the AXI bus frequency if we can. */
	if (strt_s->axiclk_khz != tgt_s->axiclk_khz) {
		res = ebi1_clk_set_min_rate_dx(CLKVOTE_ACPUCLK,
						tgt_s->axiclk_khz * 1000);
		if (res < 0)
			pr_warning("Setting AXI min rate failed (%d)\n", res);
	}

	/* Nothing else to do for power collapse if not 7x27. */
	/*if (reason == SETRATE_PC && !cpu_is_msm7x27())
		goto out;*/

	/* Disable PLLs we are not using anymore. */
	if (tgt_s->pll != ACPU_PLL_TCXO)
		plls_enabled &= ~(1 << tgt_s->pll);
	for (pll = ACPU_PLL_0; pll <= ACPU_PLL_2; pll++)
		if (plls_enabled & (1 << pll)) {
			res = pc_pll_request_dx(pll, 0);
			if (res < 0)
				pr_warning("PLL%d disable failed (%d)\n",
						pll, res);
		}

	/* Nothing else to do for power collapse. */
	if (reason == SETRATE_PC)
		goto out;

	/* Drop VDD level if we can. */
	if (tgt_s->vdd < strt_s->vdd) {
		// DXDBG(printk(KERN_INFO "%s: dropping to new vdd %d\n", __FUNCTION__, tgt_s->vdd);)
		res = acpuclk_set_vdd_level_dx(tgt_s->vdd);
		if (res < 0)
			pr_warning("Unable to drop ACPU vdd (%d)\n", res);
	}

	/*dprintk("ACPU speed change complete\n");*/
out:
	//DXDBG(printk(KERN_INFO "%s: out\n", __FUNCTION__);)
	if (reason == SETRATE_CPUFREQ)
		mutex_unlock(&drv_state_dx->lock);
	return rc;
}


// set scaling max freq for current gvernor
static void set_governor_max_freq(int max_freq) {
	unsigned int cpufreq_cpu_data_base = *(unsigned int *) ofs_cpufreq_cpu_data;
	patch(cpufreq_cpu_data_base + 0x20, max_freq);		// policy max freq
	patch(cpufreq_cpu_data_base + 0x24, max_freq);		// max freq
}

// relink the table with up/down links
static void recreate_up_down_fields(void) {
	int i, step_idx;
	
#define cur_freq standard_clocks[i].a11clk_khz
#define step_freq standard_clocks[step_idx].a11clk_khz
#define cur_pll standard_clocks[i].pll
#define step_pll standard_clocks[step_idx].pll
	
	for (i = 0; standard_clocks[i].a11clk_khz; i++) {
		// Calculate max "up" step for each destination PLL 
		step_idx = i + 1;
		while (step_freq && (step_freq - cur_freq)
					<= drv_state_dx->max_speed_delta_khz) {
			standard_clocks[i].up[step_pll] = &standard_clocks[step_idx];
			step_idx++;
		}
		if (step_idx == (i + 1) && step_freq) {
			pr_crit("Delta between freqs %u KHz and %u KHz is"
				" too high!\n", cur_freq, step_freq);
			BUG();
		}

		// Calculate max "down" step for each destination PLL 
		step_idx = i - 1;
		while (step_idx >= 0 && (cur_freq - step_freq)
					<= drv_state_dx->max_speed_delta_khz) {
			standard_clocks[i].down[step_pll] = &standard_clocks[step_idx];
			step_idx--;
		}
		
		if (step_idx == (i - 1) && i > 0) {
			pr_crit("Delta between freqs %u KHz and %u KHz is"
				" too high!\n", cur_freq, step_freq);
			BUG();
		}
		standard_clocks[i].lpj = standard_clocks[i].a11clk_khz * 5;
		
		if (standard_clocks[i].a11clk_khz == 691200) {		// default 691.2MHz 
			drv_state_dx->current_speed = &standard_clocks[i];
		}		
	}
}


// regenerate cpufreq_stat tables
static void recreate_cpufreq_stat_tables(struct cpufreq_stats *stat, int count) {
	int i;
	int alloc_size = count * sizeof(int) + count * sizeof(cputime64_t) + count * count * sizeof(int);
	
	DXDBG(printk(KERN_ERR MODULE_NAME": regenerating stat table. count=%d\n", count);)
	
	stat->max_state = count;
	if (stat->time_in_state) {
		DXDBG(printk(KERN_ERR MODULE_NAME": freeing memory at %8X\n", (unsigned int) stat->time_in_state);)
		kfree(stat->time_in_state);			// free old mems
	}
	stat->time_in_state = kzalloc(alloc_size, GFP_KERNEL);
	DXDBG(printk(KERN_ERR MODULE_NAME": allocated memory at %8X\n", (unsigned int) stat->time_in_state);)
	stat->freq_table = (unsigned int *)(stat->time_in_state + count);
	stat->trans_table = stat->freq_table + count;
	
	// regenerate freq_table field...
	for (i = 0; freq_table_dx[i].frequency != CPUFREQ_TABLE_END; i++) {
		stat->freq_table[i] = freq_table_dx[i].frequency;
		DXDBG(printk(KERN_ERR MODULE_NAME": adding %d to stat freq_table\n", freq_table_dx[i].frequency);)
	}
}

// add new ratio into the freq tables
static void add_ratio(int new_ratio) {
	unsigned int cpufreq_cpu_data_base;

	int new_speed = new_ratio * 19200;
	
	num_freq_added++;
	
	printk(KERN_ERR MODULE_NAME": adding ratio %2X (clock %d)\n", new_ratio, new_speed);

	// info from 600000, except loops per jiffies
	memcpy(&standard_clocks[8 + num_freq_added], &standard_clocks[8], sizeof(struct clkctl_acpu_speed_dx));
	standard_clocks[8 + num_freq_added].a11clk_khz = new_speed;
	standard_clocks[8 + num_freq_added].lpj = new_speed * 5;
	// end of table
	memset(&standard_clocks[9 + num_freq_added], 0, sizeof(struct clkctl_acpu_speed_dx));

	// add max freq in freq table 
	freq_table_dx[4 + num_freq_added].index = 4 + num_freq_added;
	freq_table_dx[4 + num_freq_added].frequency = new_speed;
	// end of table
	freq_table_dx[5 + num_freq_added].index = 5 + num_freq_added;
	freq_table_dx[5 + num_freq_added].frequency = CPUFREQ_TABLE_END;

	// patch cpufreq_cpu_data
	cpufreq_cpu_data_base = *(unsigned int *) ofs_cpufreq_cpu_data;
	patch(cpufreq_cpu_data_base + 0x10, new_speed);		// cpuinfo.max freq

	// patch cpufreq_stat
	cpufreq_stat_dx->state_num++;		// make it applicable
	// regenerate cpufreq_stat tables...
	recreate_cpufreq_stat_tables(cpufreq_stat_dx, cpufreq_stat_dx->state_num);
	// and fix up/down links also
	recreate_up_down_fields();
}


// handle read requests to /proc/x10minioc 
int procfile_read(char *buffer,
	      char **buffer_location,
	      off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;
	int i, j;
	char c;
	
	if (offset > 0) {
		ret  = 0;
	} else {
		if (!raw_dump) {
			ret = sprintf(buffer, "X8OC internals:\n"
						"ocratio %08X\n"
						"dump_st %08X\n"
						"dump_sz %d\n",
						x8_oc_ratio,
						dump_addr, dump_size);
			// dump the pll table
			if (standard_clocks != NULL) {
				ret = sprintf(buffer, "%sClocks:\n", buffer);
				for (i = 0; standard_clocks[i].a11clk_khz; i++) {
					ret = sprintf(buffer, "%s%6d\t%6d\t%6d\t%6d\t%6d\t%6d\t%6d\t%6d\t%6d\t%8lu\n", buffer,
						standard_clocks[i].use_for_scaling,
						standard_clocks[i].a11clk_khz,
						standard_clocks[i].pll,
						standard_clocks[i].a11clk_src_sel,
						standard_clocks[i].a11clk_src_div,
						standard_clocks[i].ahbclk_khz,
						standard_clocks[i].ahbclk_div,
						standard_clocks[i].vdd,
						standard_clocks[i].axiclk_khz,
						standard_clocks[i].lpj);
				}
			}
			// we have a memory dump request?
			if (dump_addr > 0 && dump_size > 0) {
				j = 0;
				ret = sprintf(buffer, "%sdump data:\n%08X : ", buffer, dump_addr);
				for (i = dump_addr; i < dump_addr + dump_size; i++) {
					c = *(char *)i;
					ret = sprintf(buffer, "%s%02X", buffer, c);
					if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c > '0' && c <'9'))) c = ' ';
					ret = sprintf(buffer, "%s%c ", buffer, c);
					j++;
					if (j % 16 == 0 && i < dump_addr + dump_size - 2) {
						ret = sprintf(buffer, "%s\n%08X : ", buffer, i+1);
					}
				}
				ret = sprintf(buffer, "%s\n", buffer);
			}
		}
		else {		// raw binary dumping here
			ret = dump_size;
			memcpy(buffer, (void*)dump_addr, dump_size);
		}
	}
	return ret;
}

// handle write requests to /proc/x10minioc
// retrieve commands from user land....
int procfile_write(struct file *file, const char *buffer, unsigned long count,
		   void *data)
{
	int i;
	unsigned int write_addr;
	unsigned int write_value;
	int procfs_buffer_size;
	/* get buffer size */
	procfs_buffer_size = count;
	if (procfs_buffer_size > 100 ) {
		procfs_buffer_size = 100;
	}
	
	/* examine the buffer */
	write_value=0;
	if (buffer[0] == 'w') {				// write command
		for (i=1; i <strlen(buffer); i++) {
			if (buffer[i] == ' ') {
				write_value = simple_strtol(&buffer[i+1], NULL, 10);
			}
		}
		write_addr = _httoi(&buffer[1]);
		*(unsigned int *)write_addr = write_value;
	}
	else if (buffer[0] == 'v') {			// set voltage command
		write_addr = _httoi(&buffer[1]);	// first param : freq, second param : vdd
		for (i=1; i < strlen(buffer); i++) {
			if (buffer[i] == ' ') {
				write_value = simple_strtol(&buffer[i+1], NULL, 10);
			}
		}
		
		for (i=0; standard_clocks[i].a11clk_khz; i++) {
			if (standard_clocks[i].a11clk_khz == write_addr) {
				// set it
				standard_clocks[i].vdd = write_value;
			}
		}
	}
	else if (buffer[0] == 'x') {			// set oc ratio command
		write_value = _httoi(&buffer[1]);
		set_governor_max_freq(write_value * 19200);
	}
	else if (buffer[0] == 'r') {			// set raw dump command
		dump_size = 256;
		for (i=1; i <strlen(buffer); i++) {
			if (buffer[i] == ' ') {
				dump_size = simple_strtol(&buffer[i+1], NULL, 10);
			}
		}
		dump_addr = _httoi(&buffer[1]);
		raw_dump = 1;
	}	
	else {						// set dump command : default
		raw_dump = 0;
		dump_size = 256;
		for (i=0; i <strlen(buffer); i++) {
			if (buffer[i] == ' ') {
				dump_size = simple_strtol(&buffer[i+1], NULL, 10);
			}
		}
		dump_addr = _httoi(buffer);
	}
	return procfs_buffer_size;
}


// init module
static int __init x10minioc_init(void)
{
	printk(KERN_INFO MODULE_NAME ": Module loaded. Built for target device: " DEVICE_NAME "\n");
	
	// procfs init
	x10minioc_proc = create_proc_entry(PROCFS_NAME, 0644, NULL);
	if (x10minioc_proc == NULL) {
		remove_proc_entry(PROCFS_NAME, NULL);
		printk(KERN_ALERT "Error: Could not initialize /proc/%s\n", PROCFS_NAME);
		return -ENOMEM;
	}

	x10minioc_proc->read_proc = procfile_read;
	x10minioc_proc->write_proc = procfile_write;
	x10minioc_proc->owner = THIS_MODULE;
	x10minioc_proc->mode = S_IFREG | S_IRUGO;
	x10minioc_proc->uid = 0;
	x10minioc_proc->gid = 0;
	x10minioc_proc->size = 0;

	printk(KERN_INFO MODULE_NAME ": Created procfs at /proc/" PROCFS_NAME "\n");

	// our 'GetProcAddress' :D
	kallsyms_lookup_name_dx = (void*) OFS_KALLSYMS_LOOKUP_NAME;

	// look for other offsets
	standard_clocks = (void*) kallsyms_lookup_name_dx("pll0_960_pll1_245_pll2_1200");
	drv_state_dx = (void*) kallsyms_lookup_name_dx("drv_state");
	ebi1_clk_set_min_rate_dx = (void*) kallsyms_lookup_name_dx("ebi1_clk_set_min_rate");
	acpu_freq_tbl_org = (void*) kallsyms_lookup_name_dx("acpu_freq_tbl");
	acpuclk_set_vdd_level_dx = (void*) kallsyms_lookup_name_dx("acpuclk_set_vdd_level");
	pc_pll_request_dx = (void*) kallsyms_lookup_name_dx("pc_pll_request");
	freq_table_dx = (void*) kallsyms_lookup_name_dx("freq_table");
	ofs_cpufreq_cpu_data = kallsyms_lookup_name_dx("per_cpu__cpufreq_cpu_data");
	cpufreq_stat_dx = (void*)(*(unsigned int*) kallsyms_lookup_name_dx("per_cpu__cpufreq_stats_table"));

	// get some memory for our new clock table 
	acpu_freq_tbl_dx = kzalloc(sizeof(struct clkctl_acpu_speed_dx) * 50, GFP_KERNEL);
	// copy from the old one.
	memcpy(acpu_freq_tbl_dx, standard_clocks, sizeof(struct clkctl_acpu_speed_dx) * 10);
	// point acpu_freq_tbl to it...
	patch((unsigned int)acpu_freq_tbl_org, (unsigned int) acpu_freq_tbl_dx);
	// from now on, we will use standard_clocks as the new one
	standard_clocks = acpu_freq_tbl_dx;
	
	
	
	DXDBG(
	printk(KERN_INFO MODULE_NAME ": ofs %8X standard_clocks\n", (unsigned int) standard_clocks);
	printk(KERN_INFO MODULE_NAME ": ofs %8X drv_state\n", (unsigned int) drv_state_dx);
	printk(KERN_INFO MODULE_NAME ": ofs %8X ebi1_clk_set_min_rate\n", (unsigned int) ebi1_clk_set_min_rate_dx);
	printk(KERN_INFO MODULE_NAME ": ofs %8X acpu_freq_tbl\n", (unsigned int) acpu_freq_tbl_org);
	printk(KERN_INFO MODULE_NAME ": ofs %8X acpuclk_set_vdd_level\n", (unsigned int) acpuclk_set_vdd_level_dx);
	printk(KERN_INFO MODULE_NAME ": ofs %8X pc_pll_request\n", (unsigned int) pc_pll_request_dx);
	printk(KERN_INFO MODULE_NAME ": ofs %8X freq_table\n", (unsigned int) freq_table_dx);
	printk(KERN_INFO MODULE_NAME ": ofs %8X ofs_cpufreq_cpu_data\n", (unsigned int) ofs_cpufreq_cpu_data);
	printk(KERN_INFO MODULE_NAME ": ofs %8X cpufreq_stat\n", (unsigned int) cpufreq_stat_dx);
	printk(KERN_INFO MODULE_NAME ": ofs %8X cpufreq_stat->freq_table\n", (unsigned int) cpufreq_stat_dx->freq_table);
	printk(KERN_INFO MODULE_NAME ": alloc acpu_freq_tbl_dx at %8X \n", (unsigned int) acpu_freq_tbl_dx);
	)

	// do some undervoltage
	standard_clocks[2].vdd = 0;		// a little bit under voltage for 122880
	// kinda stable
	/* standard_clocks[4].vdd = 3;		// a little bit under voltage for 245760
	standard_clocks[5].vdd = 4;		// a little bit under voltage for 320000
	standard_clocks[7].vdd = 5;		// a little bit under voltage for 480000
	standard_clocks[8].vdd = 7;		// same voltage for max */

	// test lower vdds
	standard_clocks[4].vdd = 2;		// a little bit under voltage for 245760
	standard_clocks[5].vdd = 3;		// a little bit under voltage for 320000
	standard_clocks[7].vdd = 5;		// a little bit under voltage for 480000
	standard_clocks[8].vdd = 7;		// same voltage for max


	// do our OC here
	printk(KERN_INFO MODULE_NAME ": adding more freqs\n");
	add_ratio(0x20);
	add_ratio(0x21);
	add_ratio(0x22);
	add_ratio(0x23);
	add_ratio(0x24);
	add_ratio(0x25);
	add_ratio(0x26);
	add_ratio(0x27);
	add_ratio(0x28);
	add_ratio(0x29);
	add_ratio(0x2A);
	add_ratio(0x2B);
	
	set_governor_max_freq(0x24 * 19200);		// default 691MHz...
	
	// hijack calls to acpuclk_set_rate
	patch_to_jmp(kallsyms_lookup_name_dx("acpuclk_set_rate"), &acpuclk_set_rate_dx);
	
	printk(KERN_INFO MODULE_NAME ": patching done. don't fry eggs with your phone. enjoy.\n");
	return 0;
}


// exit module - will most likely not be called
static void __exit x10minioc_exit(void)
{
	printk(KERN_INFO MODULE_NAME ": module unloaded\n");
}

module_init(x10minioc_init);
module_exit(x10minioc_exit);

MODULE_DESCRIPTION("Overclock module for Sony Ericsson X10mini / X8");
MODULE_LICENSE("GPL");
