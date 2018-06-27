// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
//
// Updated:
// 14/04/2014 by Gregory Dymare <gregd72002@gmail.com> - removed C++ dependencies
//
// Changelog:
//     2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//     2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//     2011-10-03 - added automatic Arduino version detection for ease of use
//     2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//     2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//     2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//     2011-08-02 - added support for 16-bit registers
//                - fixed incorrect Doxygen comments on some methods
//                - added timeout value for read operations (thanks mem @ Arduino forums)
//     2011-07-30 - changed read/write function structures to return success or byte counts
//                - made all methods static for multi-device memory savings
//     2011-07-28 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
//#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include "I2Cdev.h"

#include "xiicps.h"
#include "xparameters.h"

XIicPs IicInstance;

/** Default timeout value for read operations.
 * Set this to 0 to disable timeout detection.
 */
uint16_t readTimeout = 0;
/** Default constructor.
 */

int i2c_init(XIicPs *IicInstance, u16 DeviceId, u32 FsclHz)
{
	int Status;
	XIicPs_Config *ConfigPtr;	/* Pointer to configuration data */

	/*
	 * Initialize the IIC driver so that it is ready to use.
	 */
	ConfigPtr = XIicPs_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}

	Status = XIicPs_CfgInitialize(IicInstance, ConfigPtr,
					ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Set the IIC serial clock rate.
	 */
	XIicPs_SetSClk(IicInstance, FsclHz);
	return XST_SUCCESS;
}

/* Sensor Initial*/
int sensor_init()
{
	i2c_init(&IicInstance, IIC_DEVICE_ID, 100000);
	return 0;
}

/* iic write */
int i2c_wrtie_bytes(XIicPs *IicInstance,u8 i2c_slave_addr,u8 *buf,int byte_num)
{
	int Status;

		/*
		 * Send the Data.
		 */
		Status = XIicPs_MasterSendPolled(IicInstance, buf, byte_num, i2c_slave_addr);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		/*
		 * Wait until bus is idle to start another transfer.
		 */
		while (XIicPs_BusIsBusy(IicInstance));

		/*
		 * Wait for a bit of time to allow the programming to complete
		 */
		usleep(2500);

		return XST_SUCCESS;
}

/* iic read */
int i2c_read_bytes(XIicPs *IicInstance,u8 i2c_slave_addr,u8 *buf,int byte_num)
{
	int Status;

		Status = XIicPs_MasterRecvPolled(IicInstance, buf,
				byte_num, i2c_slave_addr);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		/*
		 * Wait until bus is idle to start another transfer.
		 */
		while (XIicPs_BusIsBusy(IicInstance));

		return XST_SUCCESS;
}
/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t readBytes(u8 devAddr, u8 regAddr, u8 length, u8 *data)
{
	if(i2c_wrtie_bytes(&IicInstance,devAddr,&regAddr,1) != XST_SUCCESS)
		return XST_FAILURE;

	if(i2c_read_bytes(&IicInstance,devAddr,data,length) != XST_SUCCESS)
		return XST_FAILURE;
	return XST_SUCCESS;
}


/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
int writeBytes(u8 devAddr, u8 regAddr, u8 length, u8* data)
{
	u8 buf[128];

    buf[0] = regAddr;
    memcpy(buf+1,data,length);

	if(i2c_wrtie_bytes(&IicInstance,devAddr,buf,length+1) != XST_SUCCESS)
		return XST_FAILURE;
	return XST_SUCCESS;
}


