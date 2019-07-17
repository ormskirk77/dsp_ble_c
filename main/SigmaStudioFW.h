/*
 * File:			SigmaStudioFW.h
 *
 * Description:  	SigmaStudio System Framwork macro definitions. These 
 *				macros should be implemented for your system's software.
 *
 * This software is distributed in the hope that it will be useful,
 * but is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR 
 * CONDITIONS OF ANY KIND, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * 
 * This software may only be used to program products purchased from
 * Analog Devices for incorporation by you into audio products that 
 * are intended for resale to audio product end users. This software
 * may not be distributed whole or in any part to third parties.
 *
 * Copyright � 2008 Analog Devices, Inc. All rights reserved.
 */	
 
#ifndef __SIGMASTUDIOFW_H__
#define __SIGMASTUDIOFW_H__
#include <driver/i2c.h>

/* 
 * TODO: Update for your system's data type
 */
typedef unsigned short ADI_DATA_U16;
typedef unsigned char  ADI_REG_TYPE;

#define Address_Length	2
void SIGMA_WRITE_REGISTER_BLOCK(int devAddress, int address, int length, ADI_REG_TYPE  *pData);
void SIGMA_WRITE_DELAY(int devAddress, int length, ADI_REG_TYPE  *pData );
void process_coefficient_for_i2c(float input_decimal, unsigned char coefficients_as_bytes[]);

#define SDA_PIN 21
#define SCL_PIN 22

#define HI(x)  ((x) >> 8)
#define LO(x)  ((x) & 0xFF)
#define FIXPOINT_CONVERT( _value ) _value = _value * 8388608


/* 
 * Parameter data format
 */
#define SIGMASTUDIOTYPE_FIXPOINT 	0
#define SIGMASTUDIOTYPE_INTEGER 	1

/* 
 * Write to a single Device register
 */
#define SIGMA_WRITE_REGISTER( devAddress, address, dataLength, data ) {/*TODO: implement macro or define as function*/}

/* 
 * TODO: CUSTOM MACRO IMPLEMENTATION
 * Write to multiple Device registers 
 */
void SIGMA_WRITE_REGISTER_BLOCK(int devAddress, int address, int length, ADI_REG_TYPE *pData )
{
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();

      // START I2C
		i2c_master_start(cmd);

      // Send Device ID
		i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, true);

      // mask,shift by one byte, and send upper address byte
		i2c_master_write_byte(cmd, HI(address), true);

      // mask and send lower address byte
		i2c_master_write_byte(cmd, LO(address), true);

	// Add extra dummy bit if this call is a safe load procedure.
		if(address==0x0810 || 0x0811 || 0x0812 || 0x0813 || 0x0814) {
			i2c_master_write_byte(cmd, 0x00, true);
		}

      // send data byte by byte
		for(int i=0 ; i<length ; i++){
			i2c_master_write_byte(cmd, *pData, true);
			pData++;
		}

	  // Send ACK bit on I2C bus

      // STOP I2C
		i2c_master_stop(cmd);

		esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
		i2c_cmd_link_delete(cmd);

		  if (ret == ESP_FAIL) {
		     printf("ESP_I2C_WRITE ERROR : %d\n",ret);
		  } else {
			  printf("ESP32 I2C worked with return: %X\n", ret);
		  }


}



/*
 * Read device registers 
 */
//#define SIGMA_READ_REGISTER( devAddress, address, length, pData ) {/*TODO: implement macro or define as function*/}

void SIGMA_READ_REGISTER(int devAddress, int address, int length, ADI_REG_TYPE *pData ){
//	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
////	Start condition,
//	i2c_master_start(cmd);
////	Chip address with R/W = 0 which is a write command
//	i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, true);
//
//  // mask,shift by one byte, and send upper address byte
//	i2c_master_write_byte(cmd, HI(address), true);
//
//  // mask and send lower address byte
//	i2c_master_write_byte(cmd, LO(address), true);
//
////	Repeat the start condition
//	i2c_master_start(cmd);
////	Send chip address with R/W = 1 which is a read command
//	i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_READ, true);
////	1701 will ACK
//
////	Send clocks and 1701 will output data
//	for(int i=0 ; i<length ; i++){
//		i2c_master_read_byte(cmd, *pData, true);
//		pData++;
//	}
//
////	Master will send an ACK but the 1701 will ignore this ACK since there is nothing it can do about it should it not arrive.
//
////	If you send another set of clocks then it will increment the address and send out another byte.
//
////	Master will ACK
//
////	this will continue until a stop condition is sent by the master.
}



void process_coefficient_for_i2c(float input_decimal, unsigned char coefficients_as_bytes[]){


	unsigned int fixed_point_num = FIXPOINT_CONVERT(input_decimal);


	coefficients_as_bytes[0] = (fixed_point_num >> 24) & 0xFF;
	coefficients_as_bytes[1] = (fixed_point_num >> 16) & 0xFF;
	coefficients_as_bytes[2] = (fixed_point_num >> 8) & 0xFF;
	coefficients_as_bytes[3] = fixed_point_num & 0xFF;


}



void SIGMA_SAFELOAD_SINGLE(int device_address, ADI_REG_TYPE *paramData){
//	Step 1: write parameter data to one of the safeload data registers: 2064 (0x0810) - 28bit
	//write the new parameter data that you want to be uploaded into the parameter RAM to the safeload data register.
//This step includes the safeload data register address AND the data byte 3 is a dummy byte. 08 10 00 00 80 00 00

	SIGMA_WRITE_REGISTER_BLOCK(device_address, 0x0810, 4, paramData);

	//	Step 2: write parameter address to one of the safeload address registers 2069 (0x0815) . Address of the parameter to be written - 10bit
	//volume address is 0x00. Write the parameter memory location to the safeload address location. 08 15 00 00
	ADI_REG_TYPE safe_load_address[2] = {0x00, 0x14};
	SIGMA_WRITE_REGISTER_BLOCK(device_address, 0x0815, 2, safe_load_address);

//	Step 3: the IST bit is set
//	0x08 0x1C 0x00 0x3C

	ADI_REG_TYPE safe_load_IST_flip[2] = {0x00, 0x3C};
	SIGMA_WRITE_REGISTER_BLOCK(device_address, 0x081C, 2, safe_load_IST_flip);
}


/*
 * Set a register field's value
 */
#define SIGMA_SET_REGSITER_FIELD( regVal, fieldVal, fieldMask, fieldShift )  \
		{ (regVal) = (((regVal) & (~(fieldMask))) | (((fieldVal) << (fieldShift)) && (fieldMask))) }
  
/*
 * Get the value of a register field
 */
#define SIGMA_GET_REGSITER_FIELD( regVal, fieldMask, fieldShift )  \
		{ ((regVal) & (fieldMask)) >> (fieldShift) }
 
/* 
 * Convert a floating-point value to SigmaDSP (5.23) fixed point format 
 *    This optional macro is intended for systems having special implementation
 *    requirements (for example: limited memory size or endianness)
 */
#define SIGMASTUDIOTYPE_FIXPOINT_CONVERT( _value ) {FIXPOINT_CONVERT(_value);}

/* 
 * Convert integer data to system compatible format
 *    This optional macro is intended for systems having special implementation
 *    requirements (for example: limited memory size or endianness)
 */
#define SIGMASTUDIOTYPE_INTEGER_CONVERT( _value ) {/*TODO: IMPLEMENT MACRO*/}

#endif

