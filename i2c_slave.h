/**
 * \file
 *
 * \brief I2C slave driver.
 * Compact I2C slave driver using Smart Mode,
 * based on the I2C slave driver from Atmel START.
 *
 * \page License
 *
 * © 2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Subject to your compliance with these terms, you may use Microchip software
 * and any derivatives exclusively with Microchip products. It is your
 * responsibility to comply with third party license terms applicable to your
 * use of third party software (including open source software) that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
 * IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT,
 * SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR
 * EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED,
 * EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
 * FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL
 * LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED
 * THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR
 * THIS SOFTWARE.
 */

#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Configurable I2C driver defines */
#define I2C_SLAVE_ADDRESS               63 // A1
#define I2C_RECEIVE_BUFFER_SIZE         1
#define I2C_SEND_BUFFER_SIZE            1

/*! \brief Transaction status defines*/
typedef enum i2c_status_enum {
	I2C_STATUS_READY               = (0x00 << 0),
	I2C_STATUS_BUSY                = (0x01 << 0),
} i2c_status_t;

/*! \brief Transaction result enumeration */
typedef enum i2c_result_enum {
	I2C_RESULT_UNKNOWN             = (0x00 << 0),
	I2C_RESULT_OK                  = (0x01 << 0),
	I2C_RESULT_BUFFER_OVERFLOW     = (0x02 << 0),
	I2C_RESULT_TRANSMIT_COLLISION  = (0x03 << 0),
	I2C_RESULT_BUS_ERROR           = (0x04 << 0),
	I2C_RESULT_FAIL                = (0x05 << 0),
	I2C_RESULT_ABORTED             = (0x06 << 0),
	I2C_RESULT_WAITING             = (0x07 << 0),
} i2c_result_t;

/*! \brief I2C slave driver struct.
 *  I2C slave struct. Holds pointer to I2C data processing routine,
 *  buffers and necessary variables.
 */
typedef struct twi_slave {
	uint8_t received_data[I2C_RECEIVE_BUFFER_SIZE]; /*!< Read data*/
	uint8_t send_data[I2C_SEND_BUFFER_SIZE];        /*!< Data to write*/
	uint8_t bytes_received;                         /*!< Number of bytes received*/
	uint8_t bytes_sent;                             /*!< Number of bytes sent*/
	i2c_status_t status;                            /*!< Status of transaction*/
	i2c_result_t result;                            /*!< Result of transaction*/
	bool abort;                                     /*!< Strobe to abort*/
} i2c_slave_t;
extern i2c_slave_t i2c_driver;

/**
 * \brief Initialize I2C interface
 * If module is configured to disabled state, the clock to the I2C is disabled
 * if this is supported by the device's clock system.
 *
 * \return Nothing
 */
void i2c_slave_init(void);

/**
 * \brief Open the I2C for communication. Enables the module if disabled.
 *
 * \return Nothing
 */
void i2c_slave_open(void);

/**
 * \brief Close the I2C for communication. Disables the module if enabled.
 * Disables address recognition.
 *
 * \return Nothing
 */
void i2c_slave_close(void);

/**
 * \brief The function called by the I2C IRQ handler.
 * Can be called in a polling loop in a polled driver.
 *
 * \return Nothing
 */
void i2c_slave_isr(void);

/**
 * \brief Read one byte from the data register of i2c_slave
 *
 * Function will not block if a character is not available, so should
 * only be called when data is available.
 *
 * \return Data read from the i2c_slave module
 */
uint8_t i2c_slave_read(void);

/**
 * \brief Write one byte to the data register of i2c_slave
 *
 * Function will not block if data cannot be safely accepted, so should
 * only be called when safe, i.e. in the read callback handler.
 *
 * \param[in] data The character to write to the I2C
 *
 * \return Nothing
 */
void i2c_slave_write(uint8_t data);

/**
 * \brief Enable address recognition in i2c_slave
 * 1. If supported by the clock system, enables the clock to the module
 * 2. Enables the I2C slave functionality  by setting the enable-bit in the HW's control register
 *
 * \return Nothing
 */
void i2c_slave_enable(void);

/**
 * \brief Send ACK to received address or data. Should
 * only be called when appropriate, i.e. in the callback handlers.
 *
 * \return Nothing
 */
void i2c_slave_send_ack(void);

/**
 * \brief Send NACK to received address or data. Should
 * only be called when appropriate, i.e. in the callback handlers.
 *
 * \return Nothing
 */
void i2c_slave_send_nack(void);


/**
 * \brief Send COMPTRANS to complete a transaction. Should
 * only be called when appropriate, i.e. in the callback handlers.
 *
 * \return Nothing
 */
void i2c_slave_send_comptrans(void);

/**
 * \brief Goto unaddressed state. Used to reset I2C HW that are aware
 * of bus state to an unaddressed state.
 *
 * \return Nothing
 */
void i2c_slave_goto_unaddressed(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_SLAVE_H */
