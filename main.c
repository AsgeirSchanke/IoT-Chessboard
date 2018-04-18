/*
 * RFID_reader_final.c
 *
 * Created: 15.04.2018 11.37.10
 * Author : Asgeir Schanke
 */ 

// Defines and includes //
#define F_CPU 3333333UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdbool.h>
#include "misc.h"
#include "RFID.h"
#include "i2c_slave.h"

#define scan_tag 1
#define get_ID 2

// Preallocation of functions and variables //
void tcd_init(void);
void evsys_init(void);
void io_init(void);
void tca_init(void);

void twi_init(void);

uint8_t id[5];
uint8_t send_data[5];
// Main Loop //
int main(void)
{
	tcd_init();
	evsys_init();
	io_init();
	tca_init();

	twi_init();

	sei();
    while (1)
    {
		uint8_t command = i2c_slave_read();

		// If master wants slave to scan RFID tag
		if (command == scan_tag)
		{
			if (rfid_handle(id))
			{	
				uint8_t i = 0;
				for (i=0;i<5;i++)
				{
					send_data[i] = id[i];
				}
				
				
				#ifdef DEBUG_ON
				/*
				* Writing to VPORTx.IN is equivalent to writing 
				* PORTx.OUTTGL, but faster
				*/
				DEBUG_PORT.IN |= DEBUG_PIN_FRAME_RECEIVED;
				#endif
			}
			else
			{
				uint8_t n = 0;
				for (n=0;n<5;n++)
				{
					send_data[n] = 0;
				}
			}			
		}

		// If master wants slave to send RFID data
		if (command == get_ID)
		{
			uint8_t t = 0;
			for (t=0;t<5;t++)
			{
				i2c_slave_write(send_data[t]);
			}	
		}
	}
}

// Functions for reading RFID Tag ( Alex's code ) //
/*
 * Initialize GPIO pins
 */
void io_init(void)
{
	/*
	 * Disable digital input buffers and enable
	 * pull-ups for unused pins.
	 *
	 * This is always recommended, as floating pins will
	 * have a negative impact on current consumption
	 */
	for (PORT_t *port = &PORTA; port <= &PORTB; port++)
	{
		for (register8_t *pinctrl = &(port->PIN0CTRL); pinctrl <= &(port->PIN7CTRL); pinctrl++)
		{
			*pinctrl = PORT_ISC_INPUT_DISABLE_gc | PORT_PULLUPEN_bm;
		}
	}

	/*
	 * Enable input buffer and disable pull-up for RFID RX pin
	 */
	PORTA.PIN3CTRL = 0; // Should be PB1 on ATtiny414

	/*
	 * Configure carrier TX pin as output
	 */
	VPORTB.DIR |= PIN0_bm; // Should be PB0 on ATtiny414


	#ifdef DEBUG_ON
	/*
	 * Configure debug pins as outputs
	 */
	DEBUG_PORT.DIR |= DEBUG_PIN_RX_CLOCK | DEBUG_PIN_RECEIVING_FRAME | DEBUG_PIN_FRAME_RECEIVED | DEBUG_PIN_RX_VAL;
	#endif
}

/*
 * Initialize 125 kHz carrier generator
 */
void tca_init(void)
{
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_FRQ_gc;
	TCA0.SINGLE.CMP0  = 12; /* F_CPU / (2 * 125E3) - 1 */
	TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;
}

/*
 * Initialize capture timer
 */
void tcd_init(void)
{
	/* Set max count */
	TCD0.CMPBCLR = 4095;

	/* Enable interrupt on events */
	TCD0.INTCTRL = TCD_TRIGA_bm | TCD_TRIGB_bm;
	/* Enable capture on falling edges on channel A - Rising on channel B*/
	TCD0.EVCTRLA = TCD_CFG_FILTER_gc | TCD_ACTION_bm | TCD_TRIGEI_bm;
	TCD0.EVCTRLB = TCD_CFG_FILTER_gc | TCD_ACTION_bm | TCD_TRIGEI_bm | TCD_EDGE_bm;

	/* Enable the counter with DIV4 pre-scaler */
	TCD0.CTRLA = TCD_CNTPRES_DIV4_gc | TCD_ENABLE_bm;
}

/*
 * Initialize the event system
 */
void evsys_init(void)
{
	/* Connect RFID RX pin (PB1) to the event system  */
	EVSYS.ASYNCCH0   = EVSYS_ASYNCCH0_PORTA_PIN3_gc;
	/* Connect the RFID input pin to TCD channel 0     */
	EVSYS.ASYNCUSER6 = EVSYS_ASYNCUSER6_ASYNCCH0_gc;
	/* Connect the RFID input pin to TCD channel 1     */
	EVSYS.ASYNCUSER7 = EVSYS_ASYNCUSER7_ASYNCCH0_gc;
}

/*
 * ISR triggered by rising / falling edges on the RFID RX pin
 *
 * Resets the timer and calls the RFID RX callback with
 * pulse width and polarity information
 */
ISR(TCD0_TRIG_vect)
{
	/* Read and clear interrupt flags */
	uint8_t intflags = TCD0.INTFLAGS;
	TCD0.INTFLAGS = intflags;

	/* Restart TCD */
	TCD0.CTRLE = TCD_RESTART_bm;

	if (intflags & TCD_TRIGA_bm)
	{
		rfid_rx_callback(TCD0.CAPTUREA,1);
	}
	else                      
	{
		rfid_rx_callback(TCD0.CAPTUREB,0);
	}
}

// Functions for TWI //
void twi_init(void)
{
	/* Switch pins to alternate TWI pin location */
	PORTMUX.CTRLB |= PORTMUX_TWI0_ALTERNATE_gc;

	/* Enable TWI driver */
	i2c_slave_init();
}

