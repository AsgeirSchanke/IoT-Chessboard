/*
 * RFID.c
 *
 * Created: 06.12.2016 16:04:47
 *  Author: M44001
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "misc.h"
#include "RFID.h"

/*
 * rx_status
 *
 * This register is updated both from main() and through ISR
 * context and must be volatile. Care must be taken to avoid
 * potential read-modify-write conflcts between ISR and main()
 *
 * Placing this register in GPIO0 allows atomic single bit
 * set/clear operations
 */
volatile rx_status_t rx_status __attribute__((io (&GPIO0)));

/*
 * Data bit reception states
 *
 * Placed in GPIO1 for optimization
 */
volatile bit_type_t bit_counter __attribute__((io (&GPIO1)));

/* Variable used as RX shift register */
uint8_t rx_byte;

/*
 * State variable used to keep track of
 * the different stages of the RFID reception cycle
 */
 volatile rx_state_t rfid_rx_state __attribute__((io (&GPIO3)));

/* Reset the receiver to start of frame detector mode */
static void reset_receiver(void)
{
	/* Reset receiver */
	rfid_rx_state = sof_detector;
	bit_counter   =  0;

	#ifdef DEBUG_ON
	DEBUG_PORT.OUT &= ~DEBUG_PIN_RECEIVING_FRAME;
	#endif
}

/*
 * Move the receiver in to data reception mode
 * and prepare for incoming RFID data
 */
static void start_data_reception(void)
{
	/* Start data reception */
	rfid_rx_state = rx_first_byte;
	bit_counter   = 0;

	/* Clears all parity counters */
	ATOMIC(rx_status.reg &=  ~(RX_STATUS_PARITY_COUNTER_ROWS_bm | RX_STATUS_PARITY_COUNTER_COLUMNS_bm));
}

/*
 * Store a bit in the rx_byte shift register
 */
static void shift_register_store_bit(uint8_t new_bit)
{
	rx_byte = (rx_byte << 1) | (new_bit & 0x1);
}

/*
 * Update row and column parity counters
 *
 * Parity counters will be toggled if bit is high
 */
static void update_parity_counters(uint8_t bit)
{
	if (bit == 1)
	{
		if (rx_status.parity_counter_rows == 1)
		{
			rx_status.parity_counter_rows = 0;
		}
		else
		{
			rx_status.parity_counter_rows = 1;
		}

		/* Column parity */
		uint8_t cur_column = (1 << (bit_counter % 5));
		ATOMIC(rx_status.parity_counter_columns ^= cur_column);
	}
}

/*
 * Checks if bit_counter is four, indicating that all
 * column parity bits have been received
 *
 * Returns 1 if column parity bits have been received
 */
static uint8_t column_parity_bits_received(void)
{
	return bit_counter == 4;
}

/*
 * Checks if any column parity errors have been detected
 * after an RFID package has been received
 *
 * Returns 1 of no errors where detected, 0 otherwise
 */
static uint8_t column_parity_check_status(void)
{
	return rx_status.parity_counter_columns == 0;
}

/*
 * Checks if the last row of received bits passed
 * the parity check
 *
 * Returns 1 if no errors where detected, 0 otherwise
 */
static uint8_t row_parity_check_status(void)
{
	return rx_status.parity_counter_rows == 0;
}

/*
 * Checks every received row of data for parity errors
 * and stores received bytes in the id[] array
 *
 * During reception of a byte the shift register is
 * used to build a data byte and the bit_counter
 * is used to keep track of the byte reception cycle
 *
 * If no parity errors are detected throughout
 * the reception of all RFID data bytes the
 * receiver status will be updated to
 * column_parity_check mode
 *
 * If parity errors are detected the reception is
 * aborted and the receiver is reset before
 * entering start of frame detector mode
 *
 * Input should be a pointer to a 5 byte id receiver
 * buffer
 */
static void receive_data_byte(uint8_t id_buff[5])
{
	switch(bit_counter)
	{
		case parity_bit_1:
			id_buff[rfid_rx_state] = rx_byte;

			/* Reset bit counter and advance rx state */
			bit_counter = 0;

			/* Decrement to support small endian */
			rfid_rx_state--;

			/* Fall through */
		case  parity_bit_0:
			if (row_parity_check_status() == 0)
			{
				reset_receiver();
			}
			break;
		case data_bit_0 :
		case data_bit_1 :
		case data_bit_2 :
		case data_bit_3 :
		case data_bit_4 :
		case data_bit_5 :
		case data_bit_6 :
		case data_bit_7 :
		default         :
			shift_register_store_bit(rx_status.received_bit);
	}
}

/*
 * Searches for 9 consecutive high bits
 *
 * Returns 1 if 9 consecutive high bits have been
 * detected, 0 otherwise
 */
static uint8_t start_of_frame_detected(void)
{
	/* Searching for 9 consecutive '1's, reset counter on '0's */
	if (rx_status.received_bit == 0)
	{
		bit_counter = 0;
	}
	else if (bit_counter == RFID_HEADER_LENGTH)
	{
		#ifdef DEBUG_ON
		DEBUG_PORT.OUT |= DEBUG_PIN_RECEIVING_FRAME;
		#endif

		return 1;
	}
	return 0;
}

/*
 * Handles the reception of incoming RFID data bits
 *
 * Function returns 1 if a full frame has been received
 * and stores the received data bytes in the id_buff[] array
 *
 * Returns 0 if there is nothing new
 * Returns 2 if the ISR detects an overrun condition
 *
 * Input: id_buff - Pointer to a 5 byte id receiver buffer
 */
uint8_t rfid_handle(uint8_t id_buf[5])
{
	if (rx_status.new_bit_flag == 1)
	{
		update_parity_counters(rx_status.received_bit);
		bit_counter++;

		switch(rfid_rx_state)
		{
			case sof_detector:
				if(start_of_frame_detected())
				{
					start_data_reception();
				}
				break;
			case column_parity_check:
				if (column_parity_bits_received())
				{
					reset_receiver();
					return column_parity_check_status();
				}
				break;
			case rx_first_byte :
			case rx_byte_1     :
			case rx_byte_2     :
			case rx_byte_3     :
			case rx_last_byte  :
			default            :
				receive_data_byte(id_buf);
		}
		rx_status.new_bit_flag = 0;
	}
	return 0;
}

void rfid_rx_callback(uint16_t pulse_width, uint8_t polarity)
{
	#ifdef DEBUG_ON
	/*
	 * Check if there where unhandled new bits
	 * when a new edge arrived, and increment an overrun_counter
	 * if this happens.
	 *
	 * If the overrun_counter is incremented during an actual RFID transaction
	 * it can inidcate that main() had insufficient time to process bit's
	 * between incoming edges on the rx pin
	 */
	static uint8_t overrun_counter = 0;
	if (rx_status.new_bit_flag == 1)
	{
		overrun_counter++;
	}
	#endif

	/* Manchester decoder */
	if (polarity == 0)
	{
		if (pulse_width > RFID_NEG_PW_THRESHOLD)
		{
			rx_status.received_bit = 0;
		}

		/*
		 * If current bit is low, this edge corresponds to a new valid bit
		 *
		 * This test is necessary because the RX_STATUS_BIT_VAL_HIGH bit may
		 * have been cleared in a previous call to rfid_rx_callback()
		 */
		if (rx_status.received_bit == 0)
		{
			rx_status.new_bit_flag = 1;

			#ifdef DEBUG_ON
			DEBUG_PORT.OUT &= ~DEBUG_PIN_RX_VAL;
			/*
			 * Writing to VPORTx.IN is equivalent to writing
			 * PORTx.OUTTGL, but faster
			 */
			DEBUG_PORT.IN  |= DEBUG_PIN_RX_CLOCK;
			#endif
		}
	}
	else
	{
		if (pulse_width > RFID_POS_PW_THRESHOLD)
		{
			rx_status.received_bit = 1;
		}

		/*
		 * If current bit is high, this edge corresponds to a new valid bit
		 *
		 * This test is necessary because the RX_STATUS_BIT_VAL_HIGH bit may
		 * have been set in a previous call to rfid_rx_callback()
		 */
		if (rx_status.received_bit == 1)
		{
			rx_status.new_bit_flag = 1;

			#ifdef DEBUG_ON
			DEBUG_PORT.OUT |= DEBUG_PIN_RX_VAL;
			/*
			 * Writing to VPORTx.IN is equivalent to writing
			 * PORTx.OUTTGL, but faster
			 */
			DEBUG_PORT.IN  |= DEBUG_PIN_RX_CLOCK;
			#endif
		}
	}
}