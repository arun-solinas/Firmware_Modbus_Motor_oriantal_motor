#ifndef MODBUS_MOTOR_H
#define MODBUS_MOTOR_H

#include "main.h"

// NOTE(rsb): Function Codes
#define MODBUS_READ_HOLDING_REGS    0x03
#define MODBUS_WRITE_SINGLE_REG     0x06
#define MODBUS_WRITE_MULTIPLE_REGS  0x10

// NOTE(rsb): Operation Types
#define CONTINUOUS_OPERATION_SPEED_CONTROL  0x0030

// NOTE(rsb): Register Addresses
#define REG_OPERATION_START    0x005A
#define REG_EXCITATION        0x007C

// NOTE(rsb): Timing Parameters (milliseconds)
#define MODBUS_RESPONSE_TIMEOUT     1000
#define MODBUS_INTER_FRAME_DELAY    100
#define MODBUS_RETRY_COUNT          3
#define MODBUS_COMMAND_DELAY        50
#define MODBUS_ERROR_RECOVERY_TIME  200

// NOTE(rsb): Motor IDs
#define DRUM_MOTOR_ID      1
#define SPOOLER_MOTOR_ID   2

// NOTE(rsb): Motor Directions
#define FORWARD_DIRECTION  1  // CW
#define REVERSE_DIRECTION  2  // CCW

// NOTE(rsb): Drum Motor Speed Levels (RPM)
#define M1_SPEED_LOW      1500
#define M1_SPEED_MID      2500
#define M1_SPEED_HIGH     3000

// NOTE(rsb): Spooler Motor Speed Levels (RPM)
#define M2_SPEED_LOW      (M1_SPEED_LOW/4)
#define M2_SPEED_MID      (M1_SPEED_MID/4)
#define M2_SPEED_HIGH     (M1_SPEED_HIGH/4)

// NOTE(rsb): Motor Parameters
#define DEFAULT_ACCELERATION    1000    // 1000ms
#define DEFAULT_DECELERATION    1000    // 1000ms
#define DEFAULT_TORQUE_LIMIT    2500   // 25.00%



// NOTE(rsb): Error Codes
#define MODBUS_OK             0
#define MODBUS_ERROR_CRC      1
#define MODBUS_ERROR_TIMEOUT  2
#define MODBUS_ERROR_INVALID  3
#define MODBUS_ERROR_BUSY     4
#define MODBUS_ERROR_SEQUENCE 5

// NOTE(rsb): Modbus State Structure
typedef struct {
    uint32_t lastCommandTime;
    uint8_t busyFlag;
    uint8_t retryCount;
    uint8_t lastSlaveID;
    uint8_t lastFunction;
    uint16_t lastRegister;
    uint16_t lastValue;
} ModbusState_t;

// NOTE(rsb): Core Modbus Functions
uint8_t Modbus_Init(void);
uint16_t Modbus_CalculateCRC(uint8_t *buffer, uint16_t length);
uint8_t Modbus_SendCommand(uint8_t *request, uint16_t length);
uint8_t Modbus_WaitResponse(uint32_t timeout);

// NOTE(rsb): Motor Control Functions
uint8_t Motor_Excitation_ON(uint8_t motorID);
uint8_t Motor_Excitation_OFF(uint8_t motorID);

// NOTE(rsb): Clockwise (Forward) Speed Control Functions
uint8_t Motor_CW_Low_Speed(uint8_t motorID);
uint8_t Motor_CW_Mid_Speed(uint8_t motorID);
uint8_t Motor_CW_High_Speed(uint8_t motorID);

// NOTE(rsb): Counter-Clockwise (Reverse) Speed Control Functions
uint8_t Motor_CCW_Low_Speed(uint8_t motorID);
uint8_t Motor_CCW_Mid_Speed(uint8_t motorID);
uint8_t Motor_CCW_High_Speed(uint8_t motorID);

// NOTE(rsb): Stop Function
uint8_t Motor_Stop(uint8_t motorID);

#endif // MODBUS_MOTOR_H