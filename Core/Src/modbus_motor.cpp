/*
* modbus_motor.cpp
*
*  Created on: Nov 23, 2024
*      Author: arunp
*/

#include "modbus_motor.h"

extern UART_HandleTypeDef huart6;


uint16_t Modbus_CalculateCRC(uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void Modbus_SendCommand(uint8_t slaveID, uint8_t functionCode, uint16_t regAddress, uint16_t value) {
    uint8_t request[8];
    request[0] = slaveID;
    request[1] = functionCode;
    request[2] = (regAddress >> 8) & 0xFF;
    request[3] = regAddress & 0xFF;
    request[4] = (value >> 8) & 0xFF;
    request[5] = value & 0xFF;
    uint16_t crc = Modbus_CalculateCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    
    // Use UART1 handle
    HAL_UART_Transmit(&huart6, request, 8, HAL_MAX_DELAY);
}

uint16_t Modbus_ReadResponse(uint8_t slaveID, uint8_t functionCode, uint16_t regAddress) {
    uint8_t response[8];
    
    // Use UART1 handle
    HAL_UART_Receive(&huart6, response, 8, HAL_MAX_DELAY);
    
    return (response[3] << 8) | response[4]; // Example response parsing
}

void Motor_Start(uint8_t slaveID) {
    Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_START_STOP, 1);
}

void Motor_Stop(uint8_t slaveID) {
    Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_START_STOP, 0);
}

void Motor_SetDirection(uint8_t slaveID, uint8_t direction) {
    Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_DIRECTION, direction);
}

void Motor_SetSpeed(uint8_t slaveID, uint16_t speed) {
    Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_SPEED, speed);
}

void Motor_SetTorqueLimit(uint8_t slaveID, uint16_t torqueLimit) {
    // Torque limit value is in percentage (0–100%)
    Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_TORQUE, torqueLimit);
}

void Motor_SetAcceleration(uint8_t slaveID, uint16_t acceleration) {
    // Set acceleration in RPM/s
    Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_ACCELERATION, acceleration);
}

void Motor_SetDeceleration(uint8_t slaveID, uint16_t deceleration) {
    // Set deceleration in RPM/s
    Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_DECELERATION, deceleration);
}


void Low_Forward_Synchronize() {
    
    Motor_Start(DRUM_MOTOR_ID);
    //Motor_Start(SPOOLER_MOTOR_ID);
    
    Motor_SetDirection(DRUM_MOTOR_ID,FORWARD_DIRECTION);
    //Motor_SetDirection(SPOOLER_MOTOR_ID, FORWARD_DIRECTION);
    Motor_SetSpeed(DRUM_MOTOR_ID,M1_SPEED_LOW);
    Motor_SetAcceleration(DRUM_MOTOR_ID,M1_ACCELERATION);
    Motor_SetTorqueLimit(DRUM_MOTOR_ID,M1_TORQUE_LIMIT);
        //Motor_SetAcceleration(SPOOLER_MOTOR_ID,M2_SPEED_LOW);
    
}


void Low_Reverse_Synchronize() {
    
    Motor_Start(DRUM_MOTOR_ID);
    Motor_Start(SPOOLER_MOTOR_ID);
    
    Motor_SetDirection(DRUM_MOTOR_ID, REVERSE_DIRECTION);
    Motor_SetDirection(SPOOLER_MOTOR_ID, REVERSE_DIRECTION);
    
    Motor_SetAcceleration(DRUM_MOTOR_ID,M1_SPEED_LOW);
    Motor_SetAcceleration(SPOOLER_MOTOR_ID,M2_SPEED_LOW);
}


void Mid_Forward_Synchronize() {
    Motor_Start(DRUM_MOTOR_ID);
    Motor_Start(SPOOLER_MOTOR_ID);
    
    Motor_SetDirection(DRUM_MOTOR_ID, FORWARD_DIRECTION);
    Motor_SetDirection(SPOOLER_MOTOR_ID, FORWARD_DIRECTION);
    
    Motor_SetAcceleration(DRUM_MOTOR_ID,M1_SPEED_MID);
    Motor_SetAcceleration(SPOOLER_MOTOR_ID,M2_SPEED_MID);
}


void Mid_Reverse_Synchronize() {
    
    Motor_Start(DRUM_MOTOR_ID);
    Motor_Start(SPOOLER_MOTOR_ID);
    
    Motor_SetDirection(DRUM_MOTOR_ID, REVERSE_DIRECTION);
    Motor_SetDirection(SPOOLER_MOTOR_ID, REVERSE_DIRECTION);
    
    
    Motor_SetAcceleration(DRUM_MOTOR_ID,M1_SPEED_MID);
    Motor_SetAcceleration(SPOOLER_MOTOR_ID,M2_SPEED_MID);
}


void High_Forward_Synchronize() {
    
    Motor_Start(DRUM_MOTOR_ID);
    Motor_Start(SPOOLER_MOTOR_ID);
    
    Motor_SetDirection(DRUM_MOTOR_ID, FORWARD_DIRECTION);
    Motor_SetDirection(SPOOLER_MOTOR_ID, FORWARD_DIRECTION);
    
    
    Motor_SetAcceleration(DRUM_MOTOR_ID,M1_SPEED_HIGH);
    Motor_SetAcceleration(SPOOLER_MOTOR_ID,M2_SPEED_HIGH);
}


void High_Reverse_Synchronize() {
    
    Motor_Start(DRUM_MOTOR_ID);
    Motor_Start(SPOOLER_MOTOR_ID);
    
    Motor_SetDirection(DRUM_MOTOR_ID, REVERSE_DIRECTION);
    Motor_SetDirection(SPOOLER_MOTOR_ID,REVERSE_DIRECTION);
    
    
    Motor_SetAcceleration(DRUM_MOTOR_ID,M1_SPEED_HIGH);
    Motor_SetAcceleration(SPOOLER_MOTOR_ID,M2_SPEED_HIGH);
}

