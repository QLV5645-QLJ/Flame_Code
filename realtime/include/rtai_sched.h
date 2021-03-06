/*
 * Copyright (C) 1999-2003 Paolo Mantegazza <mantegazza@aero.polimi.it>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef _RTAI_SCHED_H
#define _RTAI_SCHED_H

#include <rtai.h>
#ifndef __KERNEL__
#include <sys/time.h>
#include <time.h>
#include <errno.h>
#include <rtai_types.h>
#endif /* __KERNEL__ */

#define RT_SCHED_UP   1
#define RT_SCHED_SMP  2
#define RT_SCHED_MUP  3

#define RT_SCHED_HIGHEST_PRIORITY  0
#define RT_SCHED_LOWEST_PRIORITY   0x3fffFfff
#define RT_SCHED_LINUX_PRIORITY    0x7fffFfff

#define RT_SCHED_READY        1
#define RT_SCHED_SUSPENDED    2
#define RT_SCHED_DELAYED      4
#define RT_SCHED_SEMAPHORE    8
#define RT_SCHED_SEND        16
#define RT_SCHED_RECEIVE     32
#define RT_SCHED_RPC         64
#define RT_SCHED_RETURN     128
#define RT_SCHED_MBXSUSP    256
#define RT_SCHED_SFTRDY     512

#define RT_IRQ_TASK         0
#define RT_IRQ_TASKLET      1
#define RT_IRQ_TASK_ERR     0x7FFFFFFF

struct rt_task_struct;

#ifdef __KERNEL__

#include <linux/time.h>
#include <linux/errno.h>

#include <asm/cache.h>

#define RT_TASK_MAGIC 0x9ad25f6f  // nam2num("rttask")

#ifndef __cplusplus

#include <linux/sched.h>

typedef struct rt_queue {
	struct rt_queue *prev;
	struct rt_queue *next;
	struct rt_task_struct *task;
} QUEUE;

struct mcb_t {
    void *sbuf;
    int sbytes;
    void *rbuf;
    int rbytes;
};

typedef struct rt_ExitHandler {
    /* Exit handler functions are called like C++ destructors in
       rt_task_delete(). */
    struct rt_ExitHandler *nxt;
    void (*fun) (void *arg1, int arg2);
    void *arg1;
    int   arg2;
} XHDL;

struct rt_heap_t { void *heap, *kadr, *uadr; };

typedef struct rt_task_struct {

    int *stack __attribute__ ((__aligned__ (L1_CACHE_BYTES)));
    int uses_fpu;
    int magic;
    volatile int state, running;
    unsigned long runnable_on_cpus;
    int *stack_bottom;
    volatile int priority;
    int base_priority;
    int policy;
    int sched_lock_priority;
    struct rt_task_struct *prio_passed_to;
    RTIME period;
    RTIME resume_time;
    RTIME yield_time;
    int rr_quantum;
    int rr_remaining;
    int suspdepth;
    struct rt_queue queue;
    int owndres;
    struct rt_queue *blocked_on;
    struct rt_queue msg_queue;
    int tid;	/* trace ID */
    unsigned msg;
    struct rt_queue ret_queue;
    void (*signal)(void);
    FPU_ENV fpu_reg __attribute__ ((__aligned__ (L1_CACHE_BYTES)));
    struct rt_task_struct *prev;
    struct rt_task_struct *next;
    struct rt_task_struct *tprev;
    struct rt_task_struct *tnext;
    struct rt_task_struct *rprev;
    struct rt_task_struct *rnext;

    /* Appended for calls from LINUX. */
    int *fun_args, *bstack;
    struct task_struct *lnxtsk;
    long long retval;
    char *msg_buf[2];
    int max_msg_size[2];
    char task_name[16];
    void *system_data_ptr;
    struct rt_task_struct *nextp;
    struct rt_task_struct *prevp;

    /* Added to support user specific trap handlers. */
    RT_TRAP_HANDLER task_trap_handler[RTAI_NR_TRAPS];

    /* Added from rtai-22. */
    void (*usp_signal)(void);
    volatile unsigned long pstate;
    unsigned long usp_flags;
    unsigned long usp_flags_mask;
    unsigned long force_soft;
    volatile int is_hard;

    void *trap_handler_data;
    struct rt_task_struct *linux_syscall_server; 

    /* For use by watchdog. */
    int resync_frame;

    /* For use by exit handler functions. */
    XHDL *ExitHook;

    RTIME exectime[2];
    struct mcb_t mcb;

    /* Real time heaps. */
    struct rt_heap_t heap[2];

} RT_TASK __attribute__ ((__aligned__ (L1_CACHE_BYTES)));

#else /* __cplusplus */
extern "C" {
#endif /* !__cplusplus */

int rt_task_init(struct rt_task_struct *task,
		 void (*rt_thread)(int),
		 int data,
		 int stack_size,
		 int priority,
		 int uses_fpu,
		 void(*signal)(void));

int rt_task_init_cpuid(struct rt_task_struct *task,
		       void (*rt_thread)(int),
		       int data,
		       int stack_size,
		       int priority,
		       int uses_fpu,
		       void(*signal)(void),
		       unsigned run_on_cpu);

void rt_set_runnable_on_cpus(struct rt_task_struct *task,
			     unsigned long cpu_mask);

void rt_set_runnable_on_cpuid(struct rt_task_struct *task,
			      unsigned cpuid);

void rt_set_sched_policy(struct rt_task_struct *task,
			 int policy,
			 int rr_quantum_ns);

int rt_task_delete(struct rt_task_struct *task);

int rt_get_task_state(struct rt_task_struct *task);

void rt_gettimeorig(RTIME time_orig[]);

int rt_get_timer_cpu(void);

int rt_is_hard_timer_running(void);

void rt_set_periodic_mode(void);

void rt_set_oneshot_mode(void);

RTIME start_rt_timer(int period);

#define start_rt_timer_ns(period) start_rt_timer(nano2count((period)))

void start_rt_apic_timers(struct apic_timer_setup_data *setup_mode,
			  unsigned rcvr_jiffies_cpuid);

void stop_rt_timer(void);

struct rt_task_struct *rt_whoami(void);

int rt_sched_type(void);

int rt_task_signal_handler(struct rt_task_struct *task,
			   void (*handler)(void));

int rt_task_use_fpu(struct rt_task_struct *task,
		    int use_fpu_flag);
  
void rt_linux_use_fpu(int use_fpu_flag);

void rt_preempt_always(int yes_no);

void rt_preempt_always_cpuid(int yes_no,
			     unsigned cpuid);

RTIME count2nano(RTIME timercounts);

RTIME nano2count(RTIME nanosecs);
  
RTIME count2nano_cpuid(RTIME timercounts,
		       unsigned cpuid);

RTIME nano2count_cpuid(RTIME nanosecs,
		       unsigned cpuid);
  
RTIME rt_get_time(void);

RTIME rt_get_time_cpuid(unsigned cpuid);

RTIME rt_get_time_ns(void);

RTIME rt_get_time_ns_cpuid(unsigned cpuid);

RTIME rt_get_cpu_time_ns(void);

int rt_get_prio(struct rt_task_struct *task);

int rt_get_inher_prio(struct rt_task_struct *task);

void rt_spv_RMS(int cpuid);

int rt_change_prio(struct rt_task_struct *task,
		   int priority);

void rt_sched_lock(void);

void rt_sched_unlock(void);

void rt_task_yield(void);

int rt_task_suspend(struct rt_task_struct *task);

int rt_task_suspend_if(struct rt_task_struct *task);

int rt_task_suspend_until(struct rt_task_struct *task, RTIME until);

int rt_task_suspend_timed(struct rt_task_struct *task, RTIME delay);

int rt_task_resume(struct rt_task_struct *task);

void rt_exec_linux_syscall(RT_TASK *rt_current, RT_TASK *task, void *regs);

void rt_receive_linux_syscall(RT_TASK *task, void *regs);

void rt_return_linux_syscall(RT_TASK *task, unsigned long retval);

int rt_irq_wait(unsigned irq);

int rt_irq_wait_if(unsigned irq);

int rt_irq_wait_until(unsigned irq, RTIME until);

int rt_irq_wait_timed(unsigned irq, RTIME delay);

void rt_irq_signal(unsigned irq);

int rt_request_irq_task (unsigned irq, void *handler, int type, int affine2task);

int rt_release_irq_task (unsigned irq);

int rt_task_make_periodic_relative_ns(struct rt_task_struct *task,
				      RTIME start_delay,
				      RTIME period);

int rt_task_make_periodic(struct rt_task_struct *task,
			  RTIME start_time,
			  RTIME period);

void rt_task_set_resume_end_times(RTIME resume,
				  RTIME end);

int rt_set_resume_time(struct rt_task_struct *task,
		       RTIME new_resume_time);

int rt_set_period(struct rt_task_struct *task,
		  RTIME new_period);

int rt_task_wait_period(void);

void rt_schedule(void);

RTIME next_period(void);

void rt_busy_sleep(int nanosecs);

int rt_sleep(RTIME delay);

int rt_sleep_until(RTIME time);

int rt_task_wakeup_sleeping(struct rt_task_struct *task);

struct rt_task_struct *rt_named_task_init(const char *task_name,
					  void (*thread)(int),
					  int data,
					  int stack_size,
					  int prio,
					  int uses_fpu,
					  void(*signal)(void));

struct rt_task_struct *rt_named_task_init_cpuid(const char *task_name,
						void (*thread)(int),
						int data,
						int stack_size,
						int prio,
						int uses_fpu,
						void(*signal)(void),
						unsigned run_on_cpu);

int rt_named_task_delete(struct rt_task_struct *task);

RT_TRAP_HANDLER rt_set_task_trap_handler(struct rt_task_struct *task,
					 unsigned vec,
					 RT_TRAP_HANDLER handler);

static inline RTIME timeval2count(struct timeval *t)
{
        return nano2count(t->tv_sec*1000000000LL + t->tv_usec*1000);
}

static inline void count2timeval(RTIME rt, struct timeval *t)
{
        t->tv_sec = ulldiv(count2nano(rt), 1000000000, (unsigned long *)&t->tv_usec);
        t->tv_usec /= 1000;
}

static inline RTIME timespec2count(const struct timespec *t)
{
        return nano2count(t->tv_sec*1000000000LL + t->tv_nsec);
}

static inline void count2timespec(RTIME rt, struct timespec *t)
{
        t->tv_sec = ulldiv(count2nano(rt), 1000000000, (unsigned long *)&t->tv_nsec);
}

static inline RTIME timespec2nanos(const struct timespec *t)
{
        return t->tv_sec*1000000000LL + t->tv_nsec;
}

static inline void nanos2timespec(RTIME rt, struct timespec *t)
{
        t->tv_sec = ulldiv(rt, 1000000000, (unsigned long *)&t->tv_nsec);
}

#ifdef __cplusplus
}
#else /* !__cplusplus */

/* FIXME: These calls should move to rtai_schedcore.h */

RT_TASK *rt_get_base_linux_task(RT_TASK **base_linux_task);

RT_TASK *rt_alloc_dynamic_task(void);

void rt_enq_ready_edf_task(RT_TASK *ready_task);

void rt_enq_ready_task(RT_TASK *ready_task);

int rt_renq_ready_task(RT_TASK *ready_task,
		       int priority);

void rt_rem_ready_task(RT_TASK *task);

void rt_rem_ready_current(RT_TASK *rt_current);

void rt_enq_timed_task(RT_TASK *timed_task);

void rt_rem_timed_task(RT_TASK *task);

void rt_dequeue_blocked(RT_TASK *task);

RT_TASK **rt_register_watchdog(RT_TASK *wdog,
			       int cpuid);

void rt_deregister_watchdog(RT_TASK *wdog,
			    int cpuid);

#endif /* __cplusplus */

#endif /* __KERNEL__ */

#if !defined(__KERNEL__) || defined(__cplusplus)

typedef struct rt_task_struct {
    int opaque;
} RT_TASK;

typedef struct QueueBlock {
    int opaque;
} QBLK;

typedef struct QueueHook {
    int opaque;
} QHOOK;

#endif /* !__KERNEL__ || __cplusplus */

#endif /* !_RTAI_SCHED_H */
