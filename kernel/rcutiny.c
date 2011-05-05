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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/rcupdate.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/mutex.h>
#include <linux/time.h>

/* Definition for rcupdate control block. */
struct rcu_ctrlblk rcu_ctrlblk = {
	.completed = -300,
	.rcucblist = NULL,
	.donetail = &rcu_ctrlblk.rcucblist,
	.curtail = &rcu_ctrlblk.rcucblist,
};
EXPORT_SYMBOL_GPL(rcu_ctrlblk);
struct rcu_ctrlblk rcu_bh_ctrlblk = {
	.completed = -300,
	.rcucblist = NULL,
	.donetail = &rcu_bh_ctrlblk.rcucblist,
	.curtail = &rcu_bh_ctrlblk.rcucblist,
};
EXPORT_SYMBOL_GPL(rcu_bh_ctrlblk);

#ifdef CONFIG_NO_HZ

static long rcu_dynticks_nesting = 1;

/*
 * Enter dynticks-idle mode, which is an extended quiescent state
 * if we have fully entered that mode (i.e., if the new value of
 * dynticks_nesting is zero).
 */
void rcu_enter_nohz(void)
{
	if (--rcu_dynticks_nesting == 0)
		rcu_qsctr_inc(0); /* implies rcu_bh_qsctr_inc(0) */
}

/*
 * Exit dynticks-idle mode, so that we are no longer in an extended
 * quiescent state.
 */
void rcu_exit_nohz(void)
{
	rcu_dynticks_nesting++;
}

#endif /* #ifdef CONFIG_NO_HZ */

/*
 * Helper function for rcu_qsctr_inc() and rcu_bh_qsctr_inc().
 */
static int rcu_qsctr_help(struct rcu_ctrlblk *rcp)
{
	if (rcp->rcucblist != NULL &&
	    rcp->donetail != rcp->curtail) {
		rcp->donetail = rcp->curtail;
		return 1;
	}
	return 0;
}

/*
 * Record an rcu quiescent state.  And an rcu_bh quiescent state while we
 * are at it, given that any rcu quiescent state is also an rcu_bh
 * quiescent state.  Use "+" instead of "||" to defeat short circuiting.
 */
void rcu_qsctr_inc(int cpu)
{
	if (rcu_qsctr_help(&rcu_ctrlblk) + rcu_qsctr_help(&rcu_bh_ctrlblk))
		raise_softirq(RCU_SOFTIRQ);
}

/*
 * Record an rcu_bh quiescent state.
 */
void rcu_bh_qsctr_inc(int cpu)
{
	if (rcu_qsctr_help(&rcu_bh_ctrlblk))
		raise_softirq(RCU_SOFTIRQ);
}

/*
 * Check to see if the scheduling-clock interrupt came from an extended
 * quiescent state, and, if so, tell RCU about it.
 */
void rcu_check_callbacks(int cpu, int user)
{
	if (!rcu_needs_cpu(0))
		return;	/* RCU doesn't need anything to be done. */
	if (user ||
	    (idle_cpu(cpu) &&
	     !in_softirq() &&
	     hardirq_count() <= (1 << HARDIRQ_SHIFT)))
		rcu_qsctr_inc(cpu);
	else if (!in_softirq())
		rcu_bh_qsctr_inc(cpu);
}

/*
 * Helper function for rcu_process_callbacks() that operates on the
 * specified rcu_ctrlkblk structure.
 */
static void __rcu_process_callbacks(struct rcu_ctrlblk *rcp)
{
	unsigned long flags;
	struct rcu_head *next, *list;

	/* If no RCU callbacks ready to invoke, just return. */
	if (&rcp->rcucblist == rcp->donetail)
		return;

	/* Move the ready-to-invoke callbacks to a local list. */
	local_irq_save(flags);
	rcp->completed++;
	list = rcp->rcucblist;
	rcp->rcucblist = *rcp->donetail;
	*rcp->donetail = NULL;
	if (rcp->curtail == rcp->donetail)
		rcp->curtail = &rcp->rcucblist;
	rcp->donetail = &rcp->rcucblist;
	local_irq_restore(flags);

	/* Invoke the callbacks on the local list. */
	while (list) {
		next = list->next;
		prefetch(next);
		list->func(list);
		list = next;
	}
}

/*
 * Invoke any callbacks whose grace period has completed.
 */
static void rcu_process_callbacks(struct softirq_action *unused)
{
	__rcu_process_callbacks(&rcu_ctrlblk);
	__rcu_process_callbacks(&rcu_bh_ctrlblk);
}

/*
 * Wait for a grace period to elapse.  But it is illegal to invoke
 * synchronize_rcu() from within an RCU read-side critical section.
 * Therefore, any legal call to synchronize_rcu() is a quiescent
 * state, and so on a UP system, synchronize_rcu() need do nothing.
 *
 * Cool, huh?  (Due to Josh Triplett.)
 *
 * However, we do update the grace-period counter to prevent rcutorture
 * from hammering us.
 */
void synchronize_rcu(void)
{
	unsigned long flags;

	local_irq_save(flags);
	rcu_ctrlblk.completed++;
	local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(synchronize_rcu);

/*
 * Helper function for call_rcu() and call_rcu_bh().
 */
static void __call_rcu(struct rcu_head *head,
		       void (*func)(struct rcu_head *rcu),
		       struct rcu_ctrlblk *rcp)
{
	unsigned long flags;

	head->func = func;
	head->next = NULL;
	local_irq_save(flags);
	*rcp->curtail = head;
	rcp->curtail = &head->next;
	local_irq_restore(flags);
}

/*
 * Post an RCU callback to be invoked after the end of an RCU grace
 * period.  But since we have but one CPU, that would be after any
 * quiescent state.
 */
void call_rcu(struct rcu_head *head,
	      void (*func)(struct rcu_head *rcu))
{
	__call_rcu(head, func, &rcu_ctrlblk);
}
EXPORT_SYMBOL_GPL(call_rcu);

/*
 * Post an RCU bottom-half callback to be invoked after any subsequent
 * quiescent state.
 */
void call_rcu_bh(struct rcu_head *head,
		 void (*func)(struct rcu_head *rcu))
{
	__call_rcu(head, func, &rcu_bh_ctrlblk);
}
EXPORT_SYMBOL_GPL(call_rcu_bh);

void __rcu_init(void)
{
	open_softirq(RCU_SOFTIRQ, rcu_process_callbacks);
}
