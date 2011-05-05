/*
 * Read-Copy Update mechanism for mutual exclusion, the Bloatwatch edition.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright IBM Corporation, 2008
 *
 * Author: Paul E. McKenney <paulmck <at> linux.vnet.ibm.com>
 *
 * For detailed explanation of Read-Copy Update mechanism see -
 * 		Documentation/RCU
 */

#ifndef __LINUX_TINY_H
#define __LINUX_TINY_H

#include <linux/cache.h>

/* Global control variables for rcupdate callback mechanism. */
struct rcu_ctrlblk {
	long completed; 		/* Number of last completed batch. */
	struct rcu_head *rcucblist;	/* List of pending callbacks (CBs). */
	struct rcu_head **donetail;	/* ->next pointer of last "done" CB. */
	struct rcu_head **curtail;	/* ->next pointer of last CB. */
};

extern struct rcu_ctrlblk rcu_ctrlblk;
extern struct rcu_ctrlblk rcu_bh_ctrlblk;

/*
 * Return non-zero if there is RCU work remaining to be done.
 */
static inline int rcu_needs_cpu(int cpu)
{
	return 0;
}

void rcu_qsctr_inc(int cpu);
void rcu_bh_qsctr_inc(int cpu);

#define __rcu_read_lock()	preempt_disable()
#define __rcu_read_unlock()	preempt_enable()
#define __rcu_read_lock_bh()	local_bh_disable()
#define __rcu_read_unlock_bh()	local_bh_enable()
#define __synchronize_sched	synchronize_rcu
#define call_rcu_sched		call_rcu

#define rcu_init_sched()	do { } while (0)
extern void rcu_check_callbacks(int cpu, int user);
extern void __rcu_init(void);
/* extern void rcu_restart_cpu(int cpu); */

/*
 * Return the number of grace periods.
 */
static inline long rcu_batches_completed(void)
{
	return rcu_ctrlblk.completed;
}

/*
 * Return the number of bottom-half grace periods.
 */
static inline long rcu_batches_completed_bh(void)
{
	return rcu_bh_ctrlblk.completed;
}

static inline int rcu_pending(int cpu)
{
	return 1;
}

#ifdef CONFIG_NO_HZ

extern void rcu_enter_nohz(void);
extern void rcu_exit_nohz(void);

#else /* #ifdef CONFIG_NO_HZ */

static inline void rcu_enter_nohz(void)
{
}

static inline void rcu_exit_nohz(void)
{
}

#endif /* #else #ifdef CONFIG_NO_HZ */

#endif /* __LINUX_RCUTINY_H */
