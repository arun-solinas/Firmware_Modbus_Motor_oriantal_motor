

#ifndef MODBUS_MOTOR_H
#define MODBUS_MOTOR_H

/*
 * modbus_motor.h
 *
 *  Created on: Nov 23, 2024
 *      Author: arunp
 */

#include <stdint.h>
#include "main.h"

// Modbus Function Codes
#define MODBUS_READ_HOLDING_REG  0x03 
#define MODBUS_WRITE_SINGLE_REG  0x06
#define MODBUS_WRITE_MULTI_REG   0x10

// Motor Driver Registers
#define REG_START_STOP           0x0001  // For start and Stop
#define REG_DIRECTION            0x0002  // For Directon
#define REG_SPEED                0x0003  // For Speed control
#define REG_TORQUE			   0x0102  // For Torque
#define REG_ACCELERATION		 0x0103  // For Acceleration
#define REG_DECELERATION		 0x0104  // For Decelaration
#define REG_STATUS               0x0010  // For Get Status




//MOTOR ID'S
#define DRUM_MOTOR_ID    1 // Drum Motor ID
#define SPOOLER_MOTOR_ID 2 // Spooler Motor ID

//MOTOR DIRECTION
#define FORWARD_DIRECTION 0 // CW
#define REVERSE_DIRECTION 1 // CCW

// durm motor Speed Levels RPM
#define M1_SPEED_LOW 500   //SET LOW SPPED
#define M1_SPEED_MID 70  //SET MID SPEED
#define M1_SPEED_HIGH 90 //SET HIGH SPEED

//spooler motor speed level RPM
#define M2_SPEED_LOW  (M1_SPEED_LOW/4)
#define M2_SPEED_MID  (M1_SPEED_MID/4)
#define M2_SPEED_HIGH (M1_SPEED_HIGH/4)

// Drum motor torque
#define M1_TORQUE_LIMIT 80  //TORQUE LIMIT IN %

// Spooler motor torque
#define M2_TORQUE_LIMIT 100  // TORQUE LIMIT IN %

// drum motor acceleration
#define M1_ACCELERATION 500 // ACCELARATION LIMIT

// Spooler motor acceleration
#define M2_ACCELERATION 500 // ACCELARATION LIMIT



// FunctionS

void Modbus_SendCommand(uint8_t slaveID, uint8_t functionCode, uint16_t regAddress, uint16_t value);
uint16_t Modbus_ReadResponse(uint8_t slaveID, uint8_t functionCode, uint16_t regAddress);
void Motor_Start(uint8_t slaveID);
void Motor_Stop(uint8_t slaveID);
void Motor_SetDirection(uint8_t slaveID, uint8_t direction);
void Motor_SetSpeed(uint8_t slaveID, uint16_t speed);
void Motor_SetTorqueLimit(uint8_t slaveID, uint16_t torqueLimit);
void Motor_SetAcceleration(uint8_t slaveID, uint16_t acceleration);
void Motor_SetDeceleration(uint8_t slaveID, uint16_t deceleration);
void Low_Forward_Synchronize();
void Low_Reverse_Synchronize();
void Mid_Forward_Synchronize();
void Mid_Reverse_Synchronize();
void High_Forward_Synchronize();
void High_Reverse_Synchronize();

#endif  // MODBUS_MOTOR_H
