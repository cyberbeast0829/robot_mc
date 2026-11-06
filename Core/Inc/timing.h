/*
 * timing.h
 *
 * Optional instrumentation for the 40 kHz commutation interrupt, built on the
 * Cortex-M4 DWT cycle counter (DWT->CYCCNT, clocked at HCLK = 240 MHz, i.e.
 * 4.17 ns per count).  The counter is free-running and takes one instruction to
 * read, so the ISR can be measured without spending a timer, without an
 * interrupt and without a debugger attached - it also works on a board standing
 * on the bench with nothing but power and the motor.
 *
 * There is deliberately no output path here: myprintf() blocks, so it must never
 * run in the ISR, and the values only need to be looked at.  Read the isr_timing
 * struct in the debugger's watch window (it is a plain global in .bss, so it can
 * be watched over J-Link without any code path), or add an exporter to the main
 * loop if the numbers are ever wanted on the serial console.
 *
 * Reading the numbers:
 *   - isr_timing.isr_period is the PWM update period in cycles, derived from the
 *     timer configuration (AR and REPCNT) at isr_timing_init() time.  With the
 *     clock tree and tim.c as they are this is 6000 cycles = 25 us = 40 kHz.
 *   - period_last should equal isr_period.  If it does not, either the formula
 *     and the hardware disagree (AR/REPCNT/RCR change the update rate) or the
 *     DWT counter is not running at HCLK - check period_min, which is the
 *     *measured* value, against isr_period before trusting anything else.
 *   - period_ref is the raw DWT stamp of the previous entry.  It is a free
 *     running counter value (0..2^32-1, one wrap per 17.9 s), *not* a duration:
 *     it only exists to be subtracted from the next entry's timestamp, so do
 *     not read it as a period (the endpoint layer exports the durations only).
 *   - period_max > isr_period means the handler did not finish inside its 25 us
 *     slot; a value of about twice isr_period is a completely missed PWM cycle.
 *     overrun counts how often the slot was missed and n how many cycles were
 *     measured, so overrun/n is the rate.  (overrun and period_max are updated
 *     from the same measurement in the same block, so a reading of overrun > 0
 *     with a small period_max cannot come from the same observation window.)
 *   - seg_max[] is the worst case of each stage since the last reset - that is
 *     the number that decides the loop's timing margin, and the reason the
 *     statistics are kept as min/max rather than as an average.
 *   - total_max is the whole handler (entry to exit), including the stages'
 *     overhead and the code between them.
 *   - ext_max[] isolates the two busy-waits (the SPI/DMA flag wait inside
 *     ps_sample() and the injected-sequence wait inside analog_sample()).  That
 *     split is what decides whether it is worth hiding the wait behind other
 *     work or whether the surrounding code is the real cost - a stage is only
 *     the sum of the two plus a few dozen cycles of bookkeeping.
 *   - Write 1 to reset_req to restart the statistics; the ISR clears the request
 *     at the start of the next cycle.  Do that between operating points - the
 *     sums are only there for the average, the max values are the interesting
 *     ones and they are only meaningful for the interval they cover.
 *   - A *_min field reading ISR_TIMING_MIN_UNSET (0xFFFFFFFF) has not been
 *     sampled yet.
 *   - SystemCoreClock is the same 240 MHz the DWT counts, so isr_timing_to_us()
 *     converts cycles to microseconds.
 *
 * Caveats:
 *   - This is a *core cycle* count, so it includes flash wait states and all the
 *     pipeline effects - which is exactly what the loop's budget is measured in.
 *   - The counter wraps every 2^32 / 240 MHz = 17.9 s.  Every difference taken
 *     here is far shorter than that, and they are all unsigned subtractions, so
 *     wrapping is harmless.
 *   - n (a uint32 count of cycles) wraps after about 107 s of continuous
 *     running.  The max values are unaffected; only sum/n is, so reset before
 *     reading it as an average.
 *   - Reading CYCCNT and writing the statistics costs roughly 80-100 cycles per
 *     cycle out of 6000 (~1.5%), which is acceptable to leave enabled.  Set
 *     ISR_TIMING_ENABLE to 0 to compile all of it out, or
 *     ISR_TIMING_SEGMENTS to 0 to keep just the total / period / overrun
 *     counters (about 20 cycles) and drop the per-stage breakdown.
 *   - Do not single-step to read the values: a halted core stops the counter (or
 *     at least the sequence), so the numbers only mean anything when the target
 *     is running.  Collect them, then halt.
 */

#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>
#include "n32h47x_48x.h"

/* 0 = no instrumentation at all; 0 on ISR_TIMING_SEGMENTS = totals only */
#define ISR_TIMING_ENABLE   1
#define ISR_TIMING_SEGMENTS 1

/* The handler is entered ~20 cycles late or early from cycle to cycle, so a
 * period this much past isr_period means it did not finish inside its own slot */
#define ISR_TIMING_OVERRUN_SLACK	64u

/* Sentinel for the *_min fields: 0 is a legal duration, so it cannot be used as
 * "nothing sampled yet" - the first sample after a reset overwrites this */
#define ISR_TIMING_MIN_UNSET		0xFFFFFFFFu

/* Stages of ATIM1_UP_IRQHandler(), in execution order.  Keep this in step with
 * the marks in the handler. */
enum {
	ISR_SEG_ADC = 0,	/* analog_sample() - waits for the injected ADC sequence */
	ISR_SEG_ENC,		/* ps_sample()     - the SPI + DMA encoder read */
	ISR_SEG_FAULTS,		/* check_faults() */
	ISR_SEG_FSM,		/* run_fsm()       - FOC and the state machine */
	ISR_SEG_CAN,		/* can_tx_rx() */
	ISR_SEG_COUNT
};

/* Spans measured inside a stage, with their own start/stop pair, for the places
 * where the cost is not obvious from the stage boundaries */
enum {
	ISR_EXT_ENC_WAIT = 0,	/* the SPI + DMA flag wait in ps_sample() */
	ISR_EXT_ADC_WAIT,		/* the injected-sequence wait in analog_sample() */
	ISR_EXT_COUNT
};

typedef struct{
	uint32_t isr_period;					// PWM update period, in cycles, from ATIM1 AR/REPCNT
	uint32_t reset_req;						// write 1 to restart the statistics
	uint32_t n;								// cycles measured since the last reset (wraps after ~107 s)
	uint32_t overrun;						// cycles where the handler missed its slot
	uint32_t period_ref;					// DWT stamp of the previous entry - the subtraction reference, NOT a duration
	uint32_t period_last, period_min, period_max;	// measured ISR-to-ISR period, in cycles
	uint32_t total_last, total_min, total_max;
	uint64_t total_sum;
	uint32_t seg_last[ISR_SEG_COUNT];
	uint32_t seg_min[ISR_SEG_COUNT];
	uint32_t seg_max[ISR_SEG_COUNT];
	uint64_t seg_sum[ISR_SEG_COUNT];
	uint32_t ext_last[ISR_EXT_COUNT];
	uint32_t ext_min[ISR_EXT_COUNT];
	uint32_t ext_max[ISR_EXT_COUNT];
	uint64_t ext_sum[ISR_EXT_COUNT];
} IsrTiming;

extern volatile IsrTiming isr_timing;

/* Enable the DWT cycle counter and seed the statistics.  Call once, after
 * SystemClock_Config() (SetSysClock() uses the counter for its startup delay and
 * zeroes it) and after MX_TIM1_Init() (the PWM period is read from the timer) */
void isr_timing_init(void);

static inline uint32_t isr_timing_to_us(uint32_t cycles){
	return (uint32_t)(((uint64_t)cycles*1000000u)/SystemCoreClock);
}

#if ISR_TIMING_ENABLE

static inline uint32_t isr_timing_now(void){
	return DWT_CYCCNT;
}

/* Start of the handler: keeps the ISR-to-ISR period (period_last is the
 * measured duration, period_ref the timestamp it was measured against),
 * which is what catches a missed cycle, and handles a statistics reset
 * request */
static inline void isr_timing_begin(uint32_t t){
	uint32_t period = t - isr_timing.period_ref;		// unsigned: correct across the counter wrap
	isr_timing.period_ref = t;

	if(isr_timing.reset_req){
		isr_timing.reset_req = 0;
		isr_timing.n = 0;
		isr_timing.overrun = 0;
		isr_timing.period_last = 0;
		isr_timing.period_min = ISR_TIMING_MIN_UNSET;
		isr_timing.period_max = 0;
		isr_timing.total_last = 0;
		isr_timing.total_min = ISR_TIMING_MIN_UNSET;
		isr_timing.total_max = 0;
		isr_timing.total_sum = 0;
		for(uint32_t i = 0; i < ISR_SEG_COUNT; i++){
			isr_timing.seg_last[i] = 0;
			isr_timing.seg_min[i] = ISR_TIMING_MIN_UNSET;
			isr_timing.seg_max[i] = 0;
			isr_timing.seg_sum[i] = 0;
		}
		for(uint32_t i = 0; i < ISR_EXT_COUNT; i++){
			isr_timing.ext_last[i] = 0;
			isr_timing.ext_min[i] = ISR_TIMING_MIN_UNSET;
			isr_timing.ext_max[i] = 0;
			isr_timing.ext_sum[i] = 0;
		}
		return;		// this cycle started before the reset, so its period is not counted
	}

	if(isr_timing.n){
		/* The measured entry jitter is only about +/-20 cycles (5982..6018
		 * observed), so anything past the slack means the handler did not finish
		 * inside its own slot - see ISR_TIMING_OVERRUN_SLACK */
		isr_timing.period_last = period;
		if(period > (isr_timing.isr_period + ISR_TIMING_OVERRUN_SLACK)){isr_timing.overrun++;}
		if(period < isr_timing.period_min){isr_timing.period_min = period;}
		if(period > isr_timing.period_max){isr_timing.period_max = period;}
	}
}

/* One stage finished: 'prev' is a local in the handler carrying the chain of
 * timestamps */
static inline void isr_timing_mark(uint32_t seg, uint32_t *prev){
	uint32_t now = DWT_CYCCNT;
	uint32_t d = now - *prev;
	*prev = now;
	if(d < isr_timing.seg_min[seg]){isr_timing.seg_min[seg] = d;}
	if(d > isr_timing.seg_max[seg]){isr_timing.seg_max[seg] = d;}
	isr_timing.seg_last[seg] = d;
	isr_timing.seg_sum[seg] += d;
}

/* A span inside a stage, measured with its own start/stop pair */
static inline void isr_timing_extra(uint32_t ext, uint32_t t_start){
	uint32_t d = DWT_CYCCNT - t_start;
	if(d < isr_timing.ext_min[ext]){isr_timing.ext_min[ext] = d;}
	if(d > isr_timing.ext_max[ext]){isr_timing.ext_max[ext] = d;}
	isr_timing.ext_last[ext] = d;
	isr_timing.ext_sum[ext] += d;
}

static inline void isr_timing_end(uint32_t t_entry){
	uint32_t d = DWT_CYCCNT - t_entry;
	if(d < isr_timing.total_min){isr_timing.total_min = d;}
	if(d > isr_timing.total_max){isr_timing.total_max = d;}
	isr_timing.total_last = d;
	isr_timing.total_sum += d;
	isr_timing.n++;
}

#endif /* ISR_TIMING_ENABLE */

/* The stages are marked in the handler, so they stay in place whether or not the
 * instrumentation is compiled in */
#if ISR_TIMING_ENABLE && ISR_TIMING_SEGMENTS
#define ISR_TIMING_BEGIN()	uint32_t t_entry = isr_timing_now(); uint32_t t_prev = t_entry; isr_timing_begin(t_entry)
#define ISR_TIMING_MARK(seg)	isr_timing_mark((seg), &t_prev)
#define ISR_TIMING_END()	isr_timing_end(t_entry)
#elif ISR_TIMING_ENABLE
#define ISR_TIMING_BEGIN()	uint32_t t_entry = isr_timing_now(); isr_timing_begin(t_entry)
#define ISR_TIMING_MARK(seg)	do{ }while(0)
#define ISR_TIMING_END()	isr_timing_end(t_entry)
#else
#define ISR_TIMING_BEGIN()	do{ }while(0)
#define ISR_TIMING_MARK(seg)	do{ }while(0)
#define ISR_TIMING_END()	do{ }while(0)
#endif

/* Wrap a busy-wait: ISR_TIMING_SPAN_BEGIN(t) ... ISR_TIMING_SPAN_END(ext, t) */
#if ISR_TIMING_ENABLE
#define ISR_TIMING_SPAN_BEGIN(name)	uint32_t name = isr_timing_now()
#define ISR_TIMING_SPAN_END(ext, name)	isr_timing_extra((ext), (name))
#else
#define ISR_TIMING_SPAN_BEGIN(name)	do{ }while(0)
#define ISR_TIMING_SPAN_END(ext, name)	do{ }while(0)
#endif

#endif /* TIMING_H */
