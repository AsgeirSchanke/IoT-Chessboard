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

#include <avr/io.h>
#include <stdbool.h>
#include "i2c_slave.h"

// TWI Slave Driver Struct
i2c_slave_t i2c_driver = {
	.status = I2C_STATUS_READY,
	.result = I2C_RESULT_UNKNOWN,
};

// Read Event Interrupt Handler
void i2c_slave_read_handler(void);

// Write Event Interrupt Handler
void i2c_slave_write_handler(void);

// Address Event Interrupt Handler
void i2c_slave_address_handler(void);

// Stop Event Interrupt Handler
void i2c_slave_stop_handler(void);

// Bus Collision Event Interrupt Handler
void i2c_slave_collision_handler(void);

// Bus Error Event Interrupt Handler
void i2c_slave_bus_error_handler(void);

// Transaction Finished Handler
void i2c_slave_transaction_finished(i2c_status_t result);

void i2c_slave_init()
{
	// TWI0.CTRLA = 0 << TWI_FMPEN_bp /* FM Plus Enable: disabled */
	//       | TWI_SDAHOLD_OFF_gc /* SDA hold time off */
	//       | TWI_SDASETUP_4CYC_gc; /* SDA setup time is 4 clock cycles */

	// TWI0.DBGCTRL = 0 << TWI_DBGRUN_bp; /* Debug Run: disabled */

	TWI0.SADDR = I2C_SLAVE_ADDRESS << TWI_ADDRMASK_gp /* Slave Address */
	             | 0 << TWI_ADDREN_bp;  /* General Call Recognition Enable: disabled */

	// TWI0.SADDRMASK = 0 << TWI_ADDREN_bp /* Address Mask Enable: disabled */
	//       | 0x0 << TWI_ADDRMASK_gp; /* Address Mask: 0x0 */

	TWI0.SCTRLA = 0 << TWI_APIEN_bp    /* Address/Stop Interrupt Enable: disabled */
	              | 0 << TWI_DIEN_bp   /* Data Interrupt Enable: disabled */
	              | 1 << TWI_ENABLE_bp /* Enable TWI Slave: enabled */
	              | 1 << TWI_PIEN_bp   /* Stop Interrupt Enable: enabled */
	              | 0 << TWI_PMEN_bp   /* Promiscuous Mode Enable: disabled */
	              | 1 << TWI_SMEN_bp;  /* Smart Mode Enable: enabled */
				  
	TWI0.MCTRLA = 1 << TWI_ENABLE_bp;  /* Enable TWI Master: enabled, for bus error detection */
}

void i2c_slave_open(void)
{
	TWI0.SCTRLA |= TWI_ENABLE_bm;
}

void i2c_slave_close(void)
{
	TWI0.SCTRLA &= ~TWI_ENABLE_bm;
}

void i2c_slave_isr()
{
	if(TWI0.SSTATUS & TWI_COLL_bm) {
		i2c_slave_collision_handler();
	}

	if(TWI0.SSTATUS & TWI_BUSERR_bm) {
		i2c_slave_bus_error_handler();
	}

	if(TWI0.SSTATUS & TWI_APIF_bm) {
		if(TWI0.SSTATUS & TWI_AP_bm) {
			i2c_slave_address_handler();
		} else {
			i2c_slave_stop_handler();
		}
	}

	if(TWI0.SSTATUS & TWI_DIF_bm) {
		if(TWI0.SSTATUS & TWI_DIR_bm) {
			/* Master wishes to read from slave */
			i2c_slave_write_handler();
		} else {
			/* Master wishes to write to slave */
			i2c_slave_read_handler();
		}
	}
}

uint8_t i2c_slave_read(void)
{
	return TWI0.SDATA;
}

void i2c_slave_write(uint8_t data)
{
	TWI0.SDATA = data;
}

void i2c_slave_enable(void)
{
	TWI0.SCTRLA |= TWI_ENABLE_bm;
}

void i2c_slave_send_ack(void)
{
	TWI0.SCTRLB = TWI_ACKACT_ACK_gc | TWI_SCMD_RESPONSE_gc;
}

void i2c_slave_send_nack(void)
{
	TWI0.SCTRLB = TWI_ACKACT_NACK_gc | TWI_SCMD_COMPTRANS_gc;
}

void i2c_slave_send_comptrans(void)
{
	TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
}

void i2c_slave_goto_unaddressed(void)
{
	/* Reset module */
	TWI0.SSTATUS = TWI_DIF_bm | TWI_APIF_bm;
	i2c_slave_send_comptrans();
}

// Read Event Interrupt Handler
void i2c_slave_read_handler(void)
{
	/* Enable stop interrupt. */
	TWI0.SCTRLA |= TWI_PIEN_bm;

	/* Check if buffer is full */
	if(i2c_driver.bytes_received < I2C_RECEIVE_BUFFER_SIZE) {
		/* If application aborts transmission, complete transaction */
		if(i2c_driver.abort) {
			i2c_slave_send_comptrans();
			i2c_slave_transaction_finished(I2C_RESULT_ABORTED);
			i2c_driver.abort = false;
		}

		/* Fetch data, smart mode will write command */
		i2c_driver.received_data[i2c_driver.bytes_received] = i2c_slave_read();
		i2c_driver.bytes_received++;
	} else {
		/* Buffer Overflow */
		i2c_slave_send_nack();
		i2c_slave_transaction_finished(I2C_RESULT_BUFFER_OVERFLOW);
	}

}

// Write Event Interrupt Handler
void i2c_slave_write_handler(void)
{
	/* NACK received, write transaction complete */
	if((i2c_driver.bytes_sent > 0) && (TWI0.SSTATUS & TWI_RXACK_bm)) {
		i2c_slave_send_comptrans();
		i2c_slave_transaction_finished(I2C_RESULT_OK);
	} else {
		/* ACK received, more data expected */
		if(i2c_driver.bytes_sent < I2C_SEND_BUFFER_SIZE) {
			/* Send data, smart mode will write command */
			i2c_slave_write(i2c_driver.send_data[i2c_driver.bytes_sent]);
			i2c_driver.bytes_sent++;
		} else {
			/* Buffer Overflow */
			i2c_slave_send_comptrans();
			i2c_slave_transaction_finished(I2C_RESULT_BUFFER_OVERFLOW);
		}
	}
}

// Address Event Interrupt Handler
void i2c_slave_address_handler(void)
{
	/* If application aborts transmission, complete transaction */
	if(i2c_driver.abort) {
		i2c_slave_send_comptrans();
		i2c_slave_transaction_finished(I2C_RESULT_ABORTED);
		i2c_driver.abort = false;
	} else {
		/* Disable stop interrupt */
		TWI0.SCTRLA &= ~TWI_PIEN_bm;

		/* Reset variables for new transmission */
		i2c_driver.status = I2C_STATUS_BUSY;
		i2c_driver.result = I2C_RESULT_UNKNOWN;
		i2c_driver.bytes_received = 0;
		i2c_driver.bytes_sent = 0;

		/* Send ACK command */
		i2c_slave_send_ack();
	}
}

// Stop Event Interrupt Handlers
void i2c_slave_stop_handler(void)
{
	/* Disable Stop Interrupt */
	TWI0.SCTRLA &= ~TWI_PIEN_bm;

	/* Clear APIF */
	TWI0.SSTATUS = TWI_APIF_bm;

	/* Finished, without sending new command */
	i2c_slave_transaction_finished(I2C_RESULT_OK);
}

// Bus Collision Event Interrupt Handler
void i2c_slave_collision_handler(void)
{
	i2c_driver.bytes_received = 0;
	i2c_driver.bytes_sent = 0;
	i2c_slave_transaction_finished(I2C_RESULT_TRANSMIT_COLLISION);
}

// Bus Error Event Interrupt Handler
void i2c_slave_bus_error_handler(void)
{
	i2c_driver.bytes_received = 0;
	i2c_driver.bytes_sent = 0;
	i2c_slave_transaction_finished(I2C_RESULT_BUS_ERROR);
}

// Transaction Finished Handler
void i2c_slave_transaction_finished(i2c_status_t result)
{
	i2c_driver.result = result;
	i2c_driver.status = I2C_STATUS_READY;
}
