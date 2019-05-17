/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file AS5048B.hpp
 *
 * Driver for AS5048B 
 */

#ifndef DRIVERS_AS5048B_HPP_
#define DRIVERS_AS5048B_HPP_


#include <drivers/device/i2c.h>
#include <math.h>
#include <px4_config.h>
#include <sys/types.h>
#include <perf/perf_counter.h>
//include new topic
//#include <uORB/topics/differential_pressure.h>
#include <uORB/uORB.h>

#define I2C_ADDRESS_1_AS5048B		0x40

// SENSOR RELATED:
#define AS5048B_PROG_REG 0x03
#define AS5048B_ADDR_REG 0x15
#define AS5048B_ZEROMSB_REG 0x16 //bits 0..7
#define AS5048B_ZEROLSB_REG 0x17 //bits 0..5
#define AS5048B_GAIN_REG 0xFA
#define AS5048B_DIAG_REG 0xFB
#define AS5048B_MAGNMSB_REG 0xFC //bits 0..7
#define AS5048B_MAGNLSB_REG 0xFD //bits 0..5
#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_ANGLLSB_REG 0xFF //bits 0..5
#define AS5048B_RESOLUTION 16384.0 //14 bits

// Moving Exponential Average on angle - beware heavy calculation for some Arduino boards
// This is a 1st order low pass filter
// Moving average is calculated on Sine et Cosine values of the angle to provide an extrapolated accurate angle value.
#define EXP_MOVAVG_N 5	//history length impact on moving average impact - keep in mind the moving average will be impacted by the measurement frequency too
#define EXP_MOVAVG_LOOP 1 //number of measurements before starting mobile Average - starting with a simple average - 1 allows a quick start. Value must be 1 minimum

//unit consts - just to make the units more readable
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4
#define U_GRAD 5
#define U_MOA 6
#define U_SOA 7
#define U_MILNATO 8
#define U_MILSE 9
#define U_MILRU 10


#define PATH_AS5048B "/dev/as5048b"

class AMS_AS5048B : public device::I2C
{
public:
	AMS_AS5048B(int address = I2C_ADDRESS_1_AS5048B);

private:
	int 		_initialized
	uint16_t 	_scale{0};

		//variables
	boolean		_debugFlag;
	boolean		_clockWise;
	uint8_t		_chipAddress;
	uint8_t		_addressRegVal;
	uint16_t	_zeroRegVal;
	double		_lastAngleRaw;
	double		_movingAvgExpAngle;
	double		_movingAvgExpSin;
	double		_movingAvgExpCos;
	double		_movingAvgExpAlpha;
	int			_movingAvgCountLoop;


	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void		cycle() override;
	int			collect() override;
	bool 		init_as5048b();
	static void cycle_trampoline(void *arg);

	//methods
	uint8_t		readReg8(uint8_t address);
	uint16_t	readReg16(uint8_t address); //16 bit value got from 2x8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value
	void		writeReg(uint8_t address, uint8_t value);
	double		convertAngle(int unit, double angle); //RAW, TRN, DEG, RAD, GRAD, MOA, SOA, MILNATO, MILSE, MILRU
	double		getExpAvgRawAngle(void);


	//internal
	void		toggleDebug(void); // start / stop debug through serial at anytime
	void		setClockWise(boolean cw = true); //set clockwise counting, default is false (native sensor)
	void		progRegister(uint8_t regVal); //nothing so far - manipulate the OTP register
	void		doProg(void); //progress programming slave address OTP
	void		doProgZero(void); //progress programming zero position OTP
	void		addressRegW(uint8_t regVal); //change the chip address
	uint8_t		addressRegR(void); //read chip address
	void		setZeroReg(void); //set Zero to current angle position
	void		zeroRegW(uint16_t regVal); //write Zero register value
	uint16_t	zeroRegR(void); //read Zero register value
	uint16_t	angleRegR(void); //read raw value of the angle register
	uint8_t		diagR(void); //read diagnostic register
	uint16_t	magnitudeR(void); //read current magnitude
	double		angleR(int unit = U_RAW, boolean newVal = true); //Read current angle or get last measure with unit conversion : RAW, TRN, DEG, RAD, GRAD, MOA, SOA, MILNATO, MILSE, MILRU
	uint8_t		getAutoGain(void);
	uint8_t		getDiagReg(void);

	void		updateMovingAvgExp(void); //measure the current angle and feed the Exponential Moving Average calculation
	double		getMovingAvgExp(int unit = U_RAW); //get Exponential Moving Average calculation
	void		resetMovingAvgExp(void); //reset Exponential Moving Average calculation values

};

#endif
