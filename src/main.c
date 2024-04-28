/* Hardware support */
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

/* Number of microseconds in one second */
#define US_PER_SEC (1000000UL)

/* Private types */
typedef struct
{
	/* Bitrate pre-scaler */
	uint32_t BRP;

	/* Bit segment one time, in quantas */
	uint32_t TS1;

	/* Bit segment two time, in quantas */
	uint32_t TS2;

} can_timing_t;

/* Private functions */
static void setup_peripherals(void);
static bool calc_can_timing(can_timing_t *timing, uint32_t bitrate, uint8_t sample_point);

/* CAN bitrate settings (public for easy debugger adjustment) */
uint32_t can_bitrate = 1000000;
uint8_t can_samplepoint = 75; /* % */

/* CAN ID to transmit */
uint32_t can_id = 0x123;
bool can_id_is_extended = false;
uint8_t can_dlc = 8;

/* Delay between frames (public for easy debugger adjustment) */
uint32_t interframe_delay = US_PER_SEC / 5000; /* us */

/* Public functions */
int main(void)
{
	/* Switch to external clock with PLL  */
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Setup hardware */
	setup_peripherals();

	/* Main loop */
	uint64_t counter = 0;
	uint16_t last_time = (uint16_t)timer_get_counter(TIM2);
	for (;;)
	{
		/* Wait for next transmission time */
		uint16_t curr_time = (uint16_t)timer_get_counter(TIM2);
		uint16_t time_diff = curr_time - last_time;
		if (time_diff < interframe_delay) continue;
		last_time = curr_time;

		/* Transmit frame */
		gpio_set(GPIOB, GPIO12);
		if (can_transmit(CAN1, can_id, can_id_is_extended, false, can_dlc, (uint8_t*)&counter) >= 0)
		{
			/* Advance counter */
			counter++;
		}
		gpio_clear(GPIOB, GPIO12);
	}
}

/* Private functions */
static void setup_peripherals(void)
{
	/* Enable GPIO clocks */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable peripheral clocks */
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_CAN1);

	/* Reset peripherals */
	rcc_periph_reset_pulse(RST_TIM2);
	rcc_periph_reset_pulse(RST_CAN1);

	/* Configure timing output */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

	/* Setup microsecond timer */
	timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / US_PER_SEC) - 1);
	timer_enable_counter(TIM2);

	/* Configure CAN pin: RX (input pull-up). */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
				  GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN_RX);
	gpio_set(GPIOA, GPIO_CAN_RX);

	/* Configure CAN pin: TX. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
				  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN_TX);

	/* Reset CAN controller */
	can_reset(CAN1);

	/* Init CAN controller - with automatic bus off management and FIFO transmit mode */
	can_timing_t timing;
	if (	!calc_can_timing(&timing, can_bitrate, can_samplepoint)
		 || can_init(CAN1, false, true, false, false, false, false,
					 CAN_BTR_SJW_4TQ,
					 ((uint32_t)timing.TS1 - 1) << CAN_BTR_TS1_SHIFT,
					 ((uint32_t)timing.TS2 - 1) << CAN_BTR_TS2_SHIFT,
					 timing.BRP,
					 false, false)
	   )
	{
		/* Wait here, until the user notices */
		for (;;) __asm__("nop");
	}
}

/* Useful online calculator: http://www.bittiming.can-wiki.info/ */
static bool calc_can_timing(can_timing_t *timing, uint32_t bitrate, uint8_t sample_point)
{
	/*
	** 1 sync time quanta
	** Bit segment time one, TS1[3:0] + 1, 4 bits wide (1-16 time quanta)
	** Bit segment time two, TS2[2:0] + 1, 3 bits wide (1-8 time quanta)
	** Total: 1 + 16 + 8 = 25
	*/
	const uint8_t max_tq1 = 16;
	const uint8_t max_tq2 = 8;
	const uint8_t max_tq = 1 + max_tq1 + max_tq2;

	/* Max clock divider BRP[9:0] + 1, 10 bits wide, (1-1024 divider) */
	const uint16_t max_brp = 1024;

	/* Find pre-scaler (maximising number of time quanta) */
	timing->BRP = 0;
	for (uint8_t brp = 1; brp <= max_brp; brp++)
	{
		/* For now, skip brp if it isn't an integer divisor or clock */
		if (0 != (rcc_apb1_frequency % brp)) continue;

		/* Calculate can clock given candidate brp */
		uint32_t scaled_freq = rcc_apb1_frequency / brp;

		/* Skip if bitrate cannot divide into scaled clock without remainder */
		if (0 != (scaled_freq % bitrate)) continue;

		/* Calculate tq number with this prescaler given bitrate */
		uint32_t num_tq = scaled_freq / bitrate;

		/* Check against limit */
		if (num_tq > max_tq) continue;

		/* Found divider that maximises number of tq with integer clock division */
		timing->BRP = brp;
		break;
	}

	/* Check that pre-scaler was found */
	if (0 == timing->BRP) return false;

	/* Calculate number of tq's to split between ts1 and ts2 given scaling */
	uint8_t num_tq = ((rcc_apb1_frequency / timing->BRP) / bitrate) - 1; /* sub sync tq */

	/* Calculate expected tq's */
	timing->TS1 = (((uint32_t)sample_point * num_tq) + 50) / 100; /* + 50 to round */
	timing->TS2 = num_tq - timing->TS1;

	/* Check ts1 and ts2 are valid....could shuffle sample point if they're not */
	if (	(0 == timing->TS1)
		 || (timing->TS1 > max_tq1)
		 || (0 == timing->TS2)
		 || (timing->TS2 > max_tq2)
	   )
	{
		/* Need to tweak sample point */
		return false;
	}

	/* Yey, they might not be perfect, but it should chooch */
	return true;
}
