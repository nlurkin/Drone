/*
 * MPU6050DMP.h
 *
 *  Created on: 18 août 2013
 *      Author: Nicolas
 */

#ifndef MPU6050DMP_H_
#define MPU6050DMP_H_

#include <MPU6050.h>
#include "mathClasses.h"
#include <avr/pgmspace.h>

class MPU6050DMP: public MPU6050 {
public:
	MPU6050DMP(): MPU6050(){};
	MPU6050DMP(uint8_t address): MPU6050(address){};

	// special methods for MotionApps 2.0 implementation
	uint8_t *dmpPacketBuffer;
	uint16_t dmpPacketSize;

	uint8_t dmpInitialize();
	bool dmpPacketAvailable();

	uint8_t dmpSetFIFORate(uint8_t fifoRate);
	uint8_t dmpGetFIFORate();
	uint8_t dmpGetSampleStepSizeMS();
	uint8_t dmpGetSampleFrequency();
	int32_t dmpDecodeTemperature(int8_t tempReg);

	// Register callbacks after a packet of FIFO data is processed
	//uint8_t dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
	//uint8_t dmpUnregisterFIFORateProcess(inv_obj_func func);
	uint8_t dmpRunFIFORateProcesses();

	// Setup FIFO for various output
	uint8_t dmpSendQuaternion(uint_fast16_t accuracy);
	uint8_t dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t dmpSendPacketNumber(uint_fast16_t accuracy);
	uint8_t dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
	uint8_t dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

	// Get Fixed Point data from FIFO
	uint8_t dmpGetAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet=0);
	uint8_t dmpGet6AxisQuaternion(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGet6AxisQuaternion(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGet6AxisQuaternion(Quaternion *q, const uint8_t* packet=0);
	uint8_t dmpGetRelativeQuaternion(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetRelativeQuaternion(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetRelativeQuaternion(Quaternion *data, const uint8_t* packet=0);
	uint8_t dmpGetGyro(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetGyro(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetGyro(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t dmpSetLinearAccelFilterCoefficient(float coef);
	uint8_t dmpGetLinearAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetLinearAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetLinearAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
	uint8_t dmpGetLinearAccelInWorld(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetLinearAccelInWorld(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
	uint8_t dmpGetGyroAndAccelSensor(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetGyroAndAccelSensor(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8_t* packet=0);
	uint8_t dmpGetGyroSensor(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetGyroSensor(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetGyroSensor(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t dmpGetControlData(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetTemperature(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetGravity(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetGravity(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetGravity(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
	uint8_t dmpGetUnquantizedAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetUnquantizedAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetUnquantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t dmpGetQuantizedAccel(int32_t *data, const uint8_t* packet=0);
	uint8_t dmpGetQuantizedAccel(int16_t *data, const uint8_t* packet=0);
	uint8_t dmpGetQuantizedAccel(VectorInt16 *v, const uint8_t* packet=0);
	uint8_t dmpGetExternalSensorData(int32_t *data, uint16_t size, const uint8_t* packet=0);
	uint8_t dmpGetEIS(int32_t *data, const uint8_t* packet=0);

	uint8_t dmpGetEuler(float *data, Quaternion *q);
	uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

	// Get Floating Point data from FIFO
	uint8_t dmpGetAccelFloat(float *data, const uint8_t* packet=0);
	uint8_t dmpGetQuaternionFloat(float *data, const uint8_t* packet=0);

	uint8_t dmpProcessFIFOPacket(const unsigned char *dmpData);
	uint8_t dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed=NULL);

	uint8_t dmpSetFIFOProcessedCallback(void (*func) (void));

	uint8_t dmpInitFIFOParam();
	uint8_t dmpCloseFIFO();
	uint8_t dmpSetGyroDataSource(uint8_t source);
	uint8_t dmpDecodeQuantizedAccel();
	uint32_t dmpGetGyroSumOfSquare();
	uint32_t dmpGetAccelSumOfSquare();
	void dmpOverrideQuaternion(long *q);
	uint16_t dmpGetFIFOPacketSize();
};

#endif /* MPU6050DMP_H_ */
