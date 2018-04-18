/*
 * RFID.h
 *
 * Created: 06.12.2016 16:05:01
 *  Author: M44001
 */


#ifndef RFID_H_
#define RFID_H_

/*
 * RFID Debug configuration
 */
#define DEBUG_ON
#ifdef DEBUG_ON
#define DEBUG_PORT                VPORTC     /* This port is used for debug signals if DEBUG_ON is defined                    */
#define DEBUG_PIN_RX_CLOCK        PIN0_bm    /* This pin toggles every time a new bit is received                             */
#define DEBUG_PIN_RECEIVING_FRAME PIN1_bm    /* This pin is high when a reception is ongoing                                  */
#define DEBUG_PIN_FRAME_RECEIVED  PIN2_bm    /* This pin toggles every time a an RFID frame has been successfully received    */
#define DEBUG_PIN_RX_VAL          PIN3_bm    /* This pins output value follows the value of  decoded received Manchester data */
#endif

/*
 * RFID receiver configuration
 */

#define RFID_Fc_kHz                    (125u)  /* The RFID carrier frequency in kHz                                             */
#define RFID_BAUD_RATE_DIVISOR         (64u )  /* The baud rate divisor to use: Baud rate = RFID_Fc / RFID_BAUD_RATE_DIVISOR    */
#define RFID_PULSE_WIDTH_TIME_BASE_MHz (5u  )  /* The time base (in MHz) used for pulse with measurements                       */
#define RFID_PULSE_NEG_POS_SKEW_us     (24u )  /* Pulse width skew in us. Used to account for differences in rise and fall time */


/*
 * ================= *
 * = Do not modify = *
 * ================= */

/*
 * Calculate symbol length and set the ideal decision threshold to 3/4 of the symbol length
 */
#define RFID_SYMBOL_LENGTH_us                (RFID_BAUD_RATE_DIVISOR * 1000 / RFID_Fc_kHz)
#define RFID_IDEAL_PULSE_WIDTH_THRESHOLD_us  (RFID_SYMBOL_LENGTH_us * 3 / 4)

/*
 * Calculate decision threshold with the selected time base and compensate for
 * skew in rise / fall time
 */
#define RFID_POS_PW_THRESHOLD                (RFID_IDEAL_PULSE_WIDTH_THRESHOLD_us + RFID_PULSE_NEG_POS_SKEW_us) * RFID_PULSE_WIDTH_TIME_BASE_MHz-1
#define RFID_NEG_PW_THRESHOLD                (RFID_IDEAL_PULSE_WIDTH_THRESHOLD_us - RFID_PULSE_NEG_POS_SKEW_us) * RFID_PULSE_WIDTH_TIME_BASE_MHz-1

/* Defines the RFID header length */
#define RFID_HEADER_LENGTH            9

typedef union
{
	struct
	{
		uint8_t parity_counter_columns  : 4;
		uint8_t parity_counter_rows     : 1;
		uint8_t new_bit_flag            : 1;
		uint8_t received_bit            : 1;
		uint8_t _unused					: 1;
	};
	uint8_t reg;
} rx_status_t;
#define RX_STATUS_PARITY_COUNTER_COLUMNS_bm (0xF)
#define RX_STATUS_PARITY_COUNTER_ROWS_bm    (1<<4)
#define RX_STATUS_NEW_BIT_FLAG_bm           (1<<5)
#define RX_STATUS_RECEIVED_BIT_VAL_bm       (1<<6)

typedef enum rfid_bit_type_tag
{
	data_bit_0      = 1,
	data_bit_1      = 2,
	data_bit_2      = 3,
	data_bit_3      = 4,

	parity_bit_0    = 5,

	data_bit_4      = 6,
	data_bit_5      = 7,
	data_bit_6      = 8,
	data_bit_7      = 9,

	parity_bit_1    = 10
} bit_type_t;

typedef enum
{
	sof_detector        = -2,
	rx_first_byte       =  4,
	rx_byte_1           =  3,
	rx_byte_2           =  2,
	rx_byte_3           =  1,
	rx_last_byte        =  0,
	column_parity_check = -1
} rx_state_t;

uint8_t rfid_handle(uint8_t id_buff[5]);
void rfid_rx_callback(uint16_t pulse_Width, uint8_t polarity);


#endif /* RFID_H_ */