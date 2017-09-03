/******************************************************************************
 * Software License Agreement
 *
 * Copyright (c) 2016, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holders nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share
 * modifications, enhancements or bug fixes with Infineon Technologies AG
 * (dave@infineon.com).
 *
 *****************************************************************************/

/****************************************************************
 * HEADER FILES
 ***************************************************************/
#include "Dave.h"
#include "radarsense2go_library.h"
#include <stdio.h>

XMC_RADARSENSE2GO_TIMING_t radarsense2go_timing =
{
		.t_sample_us = 710,          /* 710us -> 1.408kHz */
		.t_cycle_ms = 300,           /* 300 ms */
		.N_exponent_samples = 8     /* 2^8samples * 710us = 182ms BGT24 on-time (settle-time ignored) */
};

XMC_RADARSENSE2GO_ALG_t radarsense2go_algorithm =
{
		.hold_on_cycles = 1,      /* hold-on cycles to trigger detection */
		.trigger_det_level = 50,  /* 100 OK, was 200, trigger detection level */
		.rootcalc_enable = XMC_RADARSENSE2GO_DISABLED /* root calculation for magnitude disabled */
};

XMC_RADARSENSE2GO_POWERDOWN_t radarsense2go_powerdown =
{
		.sleep_deepsleep_enable   = XMC_RADARSENSE2GO_ENABLED, /* sleep / deepsleep enabled */
		.mainexec_enable          = XMC_RADARSENSE2GO_ENABLED, /* main exec enabled */
		.vadc_clock_gating_enable = XMC_RADARSENSE2GO_ENABLED  /* vadc clock gating enabled */
};

uint16_t g_sampling_data_I[256];
uint16_t g_sampling_data_Q[256];
uint32_t g_fft_data[128];
XMC_RADARSENSE2GO_MOTION_t g_motion = XMC_NO_MOTION_DETECT;
float g_max_frq_index;
bool start = false;

static XMC_RADARSENSE2GO_MOTION_t motion_last=XMC_NO_MOTION_DETECT;
void radarsense2go_result( uint32_t *fft_magnitude_array,
		uint16_t size_of_array_mag,
		int16_t *adc_aqc_array_I,
		int16_t *adc_aqc_array_Q,
		uint16_t size_of_array_acq,
		XMC_RADARSENSE2GO_MOTION_t motion,
		uint32_t max_frq_mag,
		uint32_t max_frq_index)
{
	/* place your application code for ISR context execution here */
	/* e.g. threshold calibration */
	memcpy(g_sampling_data_I, adc_aqc_array_I, size_of_array_acq * sizeof(uint16_t));
	memcpy(g_sampling_data_Q, adc_aqc_array_Q, size_of_array_acq * sizeof(uint16_t));
	memcpy(g_fft_data, &fft_magnitude_array[1], (size_of_array_mag - 1) * sizeof(uint32_t));

	g_motion = motion;

	if (motion == XMC_MOTION_DETECT_APPROACHING || motion == XMC_MOTION_DETECT_DEPARTING)
	{
		g_max_frq_index = max_frq_index * 5.5F; // 1408 Hz/ 2 / 256 (FFT SIZE) / 2
	}
	else
	{
		g_max_frq_index = 0.0F; // Do not show max frequency in case of no motion
	}

	return;
}

void radarsense2go_startacq(void)
{
	static uint32_t BGT24_settle;
	/* Turn BGT24 on */
	DIGITAL_IO_SetOutputLow(&BGT24);
	/* delay until BGT24 is settled */
	BGT24_settle=46875;
	while(BGT24_settle!=0)
	{
		BGT24_settle--;
		__NOP();
	}
	return;
}

void radarsense2go_endacq(void)
{
	/* BGT24 off time */
	DIGITAL_IO_SetOutputLow(&BGT24);
	return;
}

void radarsense2go_trigger(XMC_RADARSENSE2GO_MOTION_t detection_state)
{
	motion_last=detection_state;
	if (detection_state == XMC_MOTION_DETECT_APPROACHING)
	{
		DIGITAL_IO_SetOutputLow(&LED);
	}
	else
	{
		DIGITAL_IO_SetOutputHigh(&LED);
	}
	return;
}


int main(void)
{
	bool running = false;

	DAVE_Init(); /* Initialization of DAVE APPs  */
	DIGITAL_IO_SetOutputHigh(&LED);
	DIGITAL_IO_SetOutputLow(&BGT24);

	radarsense2go_init(
			radarsense2go_timing,
			radarsense2go_algorithm,
			radarsense2go_powerdown,
			&TIMER_0
	);

	radarsense2go_regcb_startacq ( radarsense2go_startacq );
	radarsense2go_regcb_endacq ( radarsense2go_endacq );
	radarsense2go_regcb_result ( radarsense2go_result );
	radarsense2go_regcb_trigger ( radarsense2go_trigger );
    start = true;  //!!!Start radar on power up!!!
	while (1)
	{
		if (running == false)
		{
			if (start == true)
			{
				running = true;
				radarsense2go_start();
			}
		}
		else
		{
			if (start == false)
			{
				running = false;
				radarsense2go_stop();
			}

			radarsense2go_set_detection_threshold(radarsense2go_algorithm.trigger_det_level);

			/* place your application code for main execution here */
			/* e.g. communication on peripherals */
			radarsense2go_exitmain(); /* only need to be called if
                               mainexec_enable is enabled during init */
		}

	}

}





