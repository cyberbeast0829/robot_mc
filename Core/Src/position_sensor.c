/*
 * position_sensor.c
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"

uint16_t spi_tx_words[2], spi_rx_words[2];

void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	spi_tx_words[0] = 0x2300;
	spi_tx_words[1] = 0x0000;
	for(int i = 0; i<n; i++){
		DMA_ClearFlag(DMA_FLAG_TC1, DMA2);
		DMA_ClearFlag(DMA_FLAG_TC2, DMA2);
		GPIO_WriteBits(ENC_CS, Bit_RESET); // CS low
		SPI_Transmit_DMA((uint8_t *)spi_tx_words, 2);
		SPI_Receive_DMA((uint8_t *)spi_rx_words, 2);
		while (DMA_GetFlagStatus(DMA_FLAG_TC2, DMA2) == RESET);
		while (DMA_GetFlagStatus(DMA_FLAG_TC1, DMA2) == RESET);
		GPIO_WriteBits(ENC_CS, Bit_SET); // CS high
		SPI_StopTransmit_DMA();
		SPI_StopReceive_DMA();
	}
}

void ps_sample(EncoderStruct * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */

	/* Shift around previous samples */
	encoder->old_angle = encoder->angle_singleturn;
	for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->angle_multiturn[i] = encoder->angle_multiturn[i-1];}
	//for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->count_buff[i] = encoder->count_buff[i-1];}
	//memmove(&encoder->angle_multiturn[1], &encoder->angle_multiturn[0], (N_POS_SAMPLES-1)*sizeof(float)); // this is much slower for some reason

	/* SPI read/write */
	// encoder->spi_tx_word = ENC_READ_WORD;
	DMA_ClearFlag(DMA_FLAG_TC1, DMA2);
	DMA_ClearFlag(DMA_FLAG_TC2, DMA2);
	GPIO_WriteBits(ENC_CS, Bit_RESET); // CS low
	SPI_Transmit_DMA((uint8_t *)spi_tx_words, 1);
	SPI_Receive_DMA((uint8_t *)spi_rx_words, 1);
	while (DMA_GetFlagStatus(DMA_FLAG_TC2, DMA2) == RESET);
	while (DMA_GetFlagStatus(DMA_FLAG_TC1, DMA2) == RESET);
	GPIO_WriteBits(ENC_CS, Bit_SET); // CS high
	SPI_StopTransmit_DMA();
	SPI_StopReceive_DMA();
	encoder->raw = spi_rx_words[0];

	/* Linearization */
	int off_1 = encoder->offset_lut[(encoder->raw)>>9];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
	encoder->count = encoder->raw + off_interp;


	/* Real angles in radians */
	encoder->angle_singleturn = ((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
	int int_angle = encoder->angle_singleturn;
	encoder->angle_singleturn = TWO_PI_F*(encoder->angle_singleturn - (float)int_angle);
	//encoder->angle_singleturn = TWO_PI_F*fmodf(((float)(encoder->count-M_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;

	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int_angle = (int)encoder->elec_angle;
	encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)int_angle);
	//encoder->elec_angle = TWO_PI_F*fmodf((encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers
	/* Rollover */
	int rollover = 0;
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	encoder->turns += rollover;
	if(!encoder->first_sample){
		encoder->turns = 0;
		if(encoder->angle_singleturn > PI_OVER_2_F){encoder->turns = -1;}
		else if(encoder->angle_singleturn < -PI_OVER_2_F){encoder->turns = 1;}
		encoder->first_sample = 1;
	}



	/* Multi-turn position */
	encoder->angle_multiturn[0] = encoder->angle_singleturn + TWO_PI_F*(float)encoder->turns;

	/* Velocity */
	/*
	// Attempt at a moving least squares.  Wasn't any better
		float m = (float)N_POS_SAMPLES;
		float w = 1.0f/m;
		float q = 12.0f/(m*m*m - m);
		float c1 = 0.0f;
		float ibar = (m - 1.0f)/2.0f;
		for(int i = 0; i<N_POS_SAMPLES; i++){
			c1 += encoder->angle_multiturn[i]*q*(i - ibar);
		}
		encoder->vel2 = -c1/dt;
*/
	//encoder->velocity = vel2
	encoder->velocity = (encoder->angle_multiturn[0] - encoder->angle_multiturn[N_POS_SAMPLES-1])/(dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;

}

void ps_print(EncoderStruct * encoder, int dt_ms){
	myprintf("Raw: %d", encoder->raw);
	myprintf("   Linearized Count: %d", encoder->count);
	myprintf("   Single Turn: %f", encoder->angle_singleturn);
	myprintf("   Multiturn: %f", encoder->angle_multiturn[0]);
	myprintf("   Electrical: %f", encoder->elec_angle);
	myprintf("   Turns:  %d\r\n", encoder->turns);
	//HAL_Delay(dt_ms);
}
