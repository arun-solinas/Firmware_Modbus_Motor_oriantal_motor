#include "modbus_motor.h"

extern UART_HandleTypeDef huart6;
static ModbusState_t modbusState = {0};

// NOTE(rsb): Core Functions Implementation
uint8_t Modbus_Init(void) {
    modbusState.lastCommandTime = 0;
    modbusState.busyFlag = 0;
    modbusState.retryCount = 0;
    modbusState.lastSlaveID = 0;
    modbusState.lastFunction = 0;
    modbusState.lastRegister = 0;
    modbusState.lastValue = 0;
    return MODBUS_OK;
}

uint16_t Modbus_CalculateCRC(uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint8_t Modbus_SendCommand(uint8_t *request, uint16_t length) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, request, length, MODBUS_RESPONSE_TIMEOUT);
    return (status == HAL_OK) ? MODBUS_OK : MODBUS_ERROR_TIMEOUT;
}

uint8_t Modbus_WaitResponse(uint32_t timeout) {
    uint32_t startTime = HAL_GetTick();
    while ((HAL_GetTick() - startTime) < timeout) {
        if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE)) {
            return MODBUS_OK;
        }
        HAL_Delay(1);
    }
    return MODBUS_ERROR_TIMEOUT;
}

// NOTE(rsb): Motor Control Functions Implementation
uint8_t Motor_Excitation_ON(uint8_t motorID) {
    uint8_t request[16] = {0};
    
    request[0] = motorID;           // Address
    request[1] = 0x10;             // Function code
    request[2] = 0x00;             // Register address high
    request[3] = 0x7C;             // Register address low
    request[4] = 0x00;             // Number of registers high
    request[5] = 0x02;             // Number of registers low
    request[6] = 0x04;             // Byte count
    request[7] = 0x00;             // Data high
    request[8] = 0x00;             // Data
    request[9] = 0x00;             // Data
    request[10] = 0x01;            // Data low (ON)
    
    uint16_t crc = Modbus_CalculateCRC(request, 11);
    request[11] = crc & 0xFF;
    request[12] = (crc >> 8) & 0xFF;
    
    return Modbus_SendCommand(request, 13);
}

uint8_t Motor_Excitation_OFF(uint8_t motorID) {
    uint8_t request[16] = {0};
    
    request[0] = motorID;
    request[1] = 0x10;
    request[2] = 0x00;
    request[3] = 0x7C;
    request[4] = 0x00;
    request[5] = 0x02;
    request[6] = 0x04;
    request[7] = 0x00;
    request[8] = 0x00;
    request[9] = 0x00;
    request[10] = 0x00;            // OFF
    
    uint16_t crc = Modbus_CalculateCRC(request, 11);
    request[11] = crc & 0xFF;
    request[12] = (crc >> 8) & 0xFF;
    
    return Modbus_SendCommand(request, 13);
}

uint8_t Motor_Speed_Control(uint8_t motorID, int16_t speed) {
    uint8_t request[40] = {0};
    
    request[0] = motorID;                // Address
    request[1] = 0x10;                   // Function code
    request[2] = 0x00;                   // Register address high
    request[3] = 0x5A;                   // Register address low
    request[4] = 0x00;                   // Number of registers high
    request[5] = 0x0E;                   // Number of registers low
    request[6] = 0x1C;                   // Byte count
    
    // Operation type (Continuous operation speed control)
    request[7] = 0x00;
    request[8] = 0x00;
    request[9] = 0x00;
    request[10] = 0x30;
    
    // Position (unused)
    request[11] = 0x00;
    request[12] = 0x00;
    request[13] = 0x00;
    request[14] = 0x00;
    
    // Speed
    if (speed >= 0) {
        request[15] = 0x00;
        request[16] = 0x00;
        request[17] = (speed >> 8) & 0xFF;
        request[18] = speed & 0xFF;
    } else {
        request[15] = 0xFF;
        request[16] = 0xFF;
        speed = -speed;
        request[17] = (speed >> 8) & 0xFF;
        request[18] = speed & 0xFF;
    }
    
    // Acceleration
    request[19] = 0x00;
    request[20] = 0x00;
    request[21] = (DEFAULT_ACCELERATION >> 8) & 0xFF;
    request[22] = DEFAULT_ACCELERATION & 0xFF;
    
    // Deceleration
    request[23] = 0x00;
    request[24] = 0x00;
    request[25] = (DEFAULT_DECELERATION >> 8) & 0xFF;
    request[26] = DEFAULT_DECELERATION & 0xFF;
    
    // Torque limit
    request[27] = 0x00;
    request[28] = 0x00;
    request[29] = (DEFAULT_TORQUE_LIMIT >> 8) & 0xFF;
    request[30] = DEFAULT_TORQUE_LIMIT & 0xFF;
    
    // Trigger
    request[31] = 0x00;
    request[32] = 0x00;
    request[33] = 0x00;
    request[34] = 0x01;
    
    uint16_t crc = Modbus_CalculateCRC(request, 35);
    request[35] = crc & 0xFF;
    request[36] = (crc >> 8) & 0xFF;
    
    return Modbus_SendCommand(request, 37);
}

// CW Speed Control Functions
uint8_t Motor_CW_Low_Speed(uint8_t motorID) {
    uint16_t speed = (motorID == DRUM_MOTOR_ID) ? M1_SPEED_LOW : M2_SPEED_LOW;
    return Motor_Speed_Control(motorID, speed);
}

uint8_t Motor_CW_Mid_Speed(uint8_t motorID) {
    uint16_t speed = (motorID == DRUM_MOTOR_ID) ? M1_SPEED_MID : M2_SPEED_MID;
    return Motor_Speed_Control(motorID, speed);
}

uint8_t Motor_CW_High_Speed(uint8_t motorID) {
    uint16_t speed = (motorID == DRUM_MOTOR_ID) ? M1_SPEED_HIGH : M2_SPEED_HIGH;
    return Motor_Speed_Control(motorID, speed);
}

// CCW Speed Control Functions
uint8_t Motor_CCW_Low_Speed(uint8_t motorID) {
    uint16_t speed = (motorID == DRUM_MOTOR_ID) ? M1_SPEED_LOW : M2_SPEED_LOW;
    return Motor_Speed_Control(motorID, -speed);
}

uint8_t Motor_CCW_Mid_Speed(uint8_t motorID) {
    uint16_t speed = (motorID == DRUM_MOTOR_ID) ? M1_SPEED_MID : M2_SPEED_MID;
    return Motor_Speed_Control(motorID, -speed);
}

uint8_t Motor_CCW_High_Speed(uint8_t motorID) {
    uint16_t speed = (motorID == DRUM_MOTOR_ID) ? M1_SPEED_HIGH : M2_SPEED_HIGH;
    return Motor_Speed_Control(motorID, -speed);
}

uint8_t Motor_Stop(uint8_t motorID) {
    return Motor_Speed_Control(motorID, 0);
}