#ifndef MODBUS_MOTOR_H
#define MODBUS_MOTOR_H

#include "main.h"

// NOTE(rsb): Modbus Function Codes

#define MODBUS_READ_HOLDING_REGS    0x03
#define MODBUS_WRITE_SINGLE_REG     0x06
#define MODBUS_WRITE_MULTIPLE_REGS  0x10

// NOTE(rsb): Register Addresses

#define REG_START_STOP     0x0100
#define REG_DIRECTION      0x0101
#define REG_SPEED          0x0102  
#define REG_TORQUE         0x0103  
#define REG_ACCELERATION   0x0104  
#define REG_DECELERATION   0x0105 
#define REG_STATUS         0x0106  

// NOTE(rsb): Timing Parameters (milliseconds)

#define MODBUS_RESPONSE_TIMEOUT     1000    // Maximum wait for response
#define MODBUS_INTER_FRAME_DELAY    100     // Time between frames
#define MODBUS_RETRY_COUNT          3       // Number of retries
#define MODBUS_COMMAND_DELAY        50      // Delay between commands
#define MODBUS_ERROR_RECOVERY_TIME  200     // Wait after error

// NOTE(rsb): Motor IDs

#define DRUM_MOTOR_ID      1  // Drum Motor ID
#define SPOOLER_MOTOR_ID   2  // Spooler Motor ID

// NOTE(rsb): Motor Directions

#define FORWARD_DIRECTION  0  // CW
#define REVERSE_DIRECTION  1  // CCW

// NOTE(rsb): Drum Motor Speed Levels (RPM)

#define M1_SPEED_LOW      500  // Low speed
#define M1_SPEED_MID      700  // Mid speed
#define M1_SPEED_HIGH     900  // High speed

// NOTE(rsb): Spooler Motor Speed Levels (RPM)

#define M2_SPEED_LOW      (M1_SPEED_LOW/4)   // Low speed
#define M2_SPEED_MID      (M1_SPEED_MID/4)   // Mid speed
#define M2_SPEED_HIGH     (M1_SPEED_HIGH/4)  // High speed

// NOTE(rsb): Motor Torque Limits (%)

#define M1_TORQUE_LIMIT   80   // Drum motor
#define M2_TORQUE_LIMIT   100  // Spooler motor

// NOTE(rsb): Motor Acceleration (RPM/s)

#define M1_ACCELERATION   500  // Drum motor
#define M2_ACCELERATION   500  // Spooler motor

// NOTE(rsb): Error Codes

#define MODBUS_OK             0
#define MODBUS_ERROR_CRC      1
#define MODBUS_ERROR_TIMEOUT  2
#define MODBUS_ERROR_INVALID  3
#define MODBUS_ERROR_BUSY     4
#define MODBUS_ERROR_SEQUENCE 5

// NOTE(rsb): Modbus State Structure

typedef struct {
    uint32_t lastCommandTime;    // Timestamp of last command
    uint8_t busyFlag;           // Communication in progress flag
    uint8_t retryCount;         // Current retry count
    uint8_t lastSlaveID;        // Last addressed slave
    uint8_t lastFunction;       // Last function code
    uint16_t lastRegister;      // Last accessed register
    uint16_t lastValue;         // Last value sent
} ModbusState_t;


// NOTE(rsb): Core Modbus Functions

uint8_t Modbus_Init(void);
uint16_t Modbus_CalculateCRC(uint8_t *buffer, uint16_t length);
uint8_t Modbus_SendCommand(uint8_t slaveID, uint8_t functionCode, uint16_t regAddress, uint16_t value);
uint16_t Modbus_ReadResponse(uint8_t slaveID, uint8_t functionCode, uint16_t *value);
uint8_t Modbus_IsReadyToSend(void);
uint8_t Modbus_WaitResponse(uint32_t timeout);
void Modbus_HandleTimeout(void);
uint8_t Modbus_RetryLastCommand(void);
uint8_t Modbus_ValidateResponse(uint8_t *response, uint8_t length, uint8_t expectedSlaveId, uint8_t expectedFunction);

// NOTE(rsb): Motor Control Functions

uint8_t Motor_Start(uint8_t slaveID);
uint8_t Motor_Stop(uint8_t slaveID);
uint8_t Motor_SetDirection(uint8_t slaveID, uint8_t direction);
uint8_t Motor_SetSpeed(uint8_t slaveID, uint16_t speed);
uint8_t Motor_SetTorqueLimit(uint8_t slaveID, uint16_t torqueLimit);
uint8_t Motor_SetAcceleration(uint8_t slaveID, uint16_t acceleration);
uint8_t Motor_SetDeceleration(uint8_t slaveID, uint16_t deceleration);
uint8_t Motor_GetStatus(uint8_t slaveID, uint16_t *status);

// NOTE(rsb): Synchronized Control Functions

uint8_t Low_Forward_Synchronize(void);
uint8_t Low_Reverse_Synchronize(void);
uint8_t Mid_Forward_Synchronize(void);
uint8_t Mid_Reverse_Synchronize(void);
uint8_t High_Forward_Synchronize(void);
uint8_t High_Reverse_Synchronize(void);
uint8_t Emergency_Stop(void);

#endif // MODBUS_MOTOR_H
