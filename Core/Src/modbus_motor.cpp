#include "modbus_motor.h"

extern UART_HandleTypeDef huart6;
static ModbusState_t modbusState = {0};

// NOTE(rsb):Initialization Functions

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

// NOTE(rsb):Core Modbus Functions

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

uint8_t Modbus_IsReadyToSend(void) {
    uint32_t currentTime = HAL_GetTick();
    if ((currentTime - modbusState.lastCommandTime) < MODBUS_INTER_FRAME_DELAY) {
        return 0;
    }
    return !modbusState.busyFlag;
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

uint8_t Modbus_ValidateResponse(uint8_t *response, uint8_t length, uint8_t expectedSlaveId, uint8_t expectedFunction) {
    
    if (response[0] != expectedSlaveId || response[1] != expectedFunction) {
        return 0;
    }
    
    uint16_t received_crc = (response[length-1] << 8) | response[length-2];
    uint16_t calculated_crc = Modbus_CalculateCRC(response, length-2);
    return (received_crc == calculated_crc);
    
}

void Modbus_HandleTimeout(void) {
    
    modbusState.busyFlag = 0;
    if (modbusState.retryCount < MODBUS_RETRY_COUNT) {
        modbusState.retryCount++;
        HAL_Delay(MODBUS_ERROR_RECOVERY_TIME);
        Modbus_RetryLastCommand();
        
    }
}

uint8_t Modbus_RetryLastCommand(void) {
    if (modbusState.lastSlaveID == 0) {
        return MODBUS_ERROR_SEQUENCE;
    }
    return Modbus_SendCommand(modbusState.lastSlaveID, 
                              modbusState.lastFunction,
                              modbusState.lastRegister, 
                              modbusState.lastValue);
}

uint8_t Modbus_SendCommand(uint8_t slaveID, uint8_t functionCode, uint16_t regAddress, uint16_t value) {
    
    
    while (!Modbus_IsReadyToSend()) {
        HAL_Delay(1);
    }
    
    
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
    
    // Store command details for potential retry
    modbusState.lastSlaveID = slaveID;
    modbusState.lastFunction = functionCode;
    modbusState.lastRegister = regAddress;
    modbusState.lastValue = value;
    modbusState.busyFlag = 1;
    
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, request, 8, MODBUS_RESPONSE_TIMEOUT);
    
    if (status != HAL_OK) {
        Modbus_HandleTimeout();
        return MODBUS_ERROR_TIMEOUT;
    }
    
    modbusState.lastCommandTime = HAL_GetTick();
    
    uint8_t responseStatus = Modbus_WaitResponse(MODBUS_RESPONSE_TIMEOUT);
    if (responseStatus != MODBUS_OK) {
        Modbus_HandleTimeout();
        return responseStatus;
    }
    
    modbusState.busyFlag = 0;
    modbusState.retryCount = 0;
    return MODBUS_OK;
}




uint16_t Modbus_ReadResponse(uint8_t slaveID, uint8_t functionCode, uint16_t *value) {
    
    uint8_t response[8];
    
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart6, response, 8, MODBUS_RESPONSE_TIMEOUT);
    
    if (status != HAL_OK) {
        Modbus_HandleTimeout();
        return MODBUS_ERROR_TIMEOUT;
    }
    
    if (!Modbus_ValidateResponse(response, 8, slaveID, functionCode)) {
        return MODBUS_ERROR_INVALID;
    }
    
    *value = (response[4] << 8) | response[5];
    modbusState.busyFlag = 0;
    return MODBUS_OK;
}


// NOTE(rsb):Motor Control Functions


uint8_t Motor_Start(uint8_t slaveID) {
    return Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_START_STOP, 1);
}

uint8_t Motor_Stop(uint8_t slaveID) {
    return Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_START_STOP, 0);
}

uint8_t Motor_SetDirection(uint8_t slaveID, uint8_t direction) {
    return Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_DIRECTION, direction);
}

uint8_t Motor_SetSpeed(uint8_t slaveID, uint16_t speed) {
    return Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_SPEED, speed);
}

uint8_t Motor_SetTorqueLimit(uint8_t slaveID, uint16_t torqueLimit) {
    if (torqueLimit > 100) torqueLimit = 100;
    return Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_TORQUE, torqueLimit);
}

uint8_t Motor_SetAcceleration(uint8_t slaveID, uint16_t acceleration) {
    return Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_ACCELERATION, acceleration);
}

uint8_t Motor_SetDeceleration(uint8_t slaveID, uint16_t deceleration) {
    return Modbus_SendCommand(slaveID, MODBUS_WRITE_SINGLE_REG, REG_DECELERATION, deceleration);
}

uint8_t Motor_GetStatus(uint8_t slaveID, uint16_t *status) {
    uint8_t result = Modbus_SendCommand(slaveID, MODBUS_READ_HOLDING_REGS, REG_STATUS, 1);
    if (result != MODBUS_OK) return result;
    return Modbus_ReadResponse(slaveID, MODBUS_READ_HOLDING_REGS, status);
}



// NOTE(rsb): Synchronized Control Functions


uint8_t Low_Forward_Synchronize(void) {
    uint8_t result;
    
    // Start drum motor with proper timing
    result = Motor_Start(DRUM_MOTOR_ID);
    if (result != MODBUS_OK) return result;
    HAL_Delay(MODBUS_COMMAND_DELAY);
    
    result = Motor_SetDirection(DRUM_MOTOR_ID, FORWARD_DIRECTION);
    if (result != MODBUS_OK) return result;
    HAL_Delay(MODBUS_COMMAND_DELAY);
    
    result = Motor_SetSpeed(DRUM_MOTOR_ID, M1_SPEED_LOW);
    if (result != MODBUS_OK) return result;
    HAL_Delay(MODBUS_COMMAND_DELAY);
    
    // Start spooler motor with proper timing
    result = Motor_Start(SPOOLER_MOTOR_ID);
    if (result != MODBUS_OK) return result;
    HAL_Delay(MODBUS_COMMAND_DELAY);
    
    result = Motor_SetDirection(SPOOLER_MOTOR_ID, FORWARD_DIRECTION);
    if (result != MODBUS_OK) return result;
    HAL_Delay(MODBUS_COMMAND_DELAY);
    
    result = Motor_SetSpeed(SPOOLER_MOTOR_ID, M2_SPEED_LOW);
    return result;
}


uint8_t Emergency_Stop(void) {
    uint8_t result;
    
    result = Motor_Stop(DRUM_MOTOR_ID);
    if (result != MODBUS_OK) return result;
    
    result = Motor_Stop(SPOOLER_MOTOR_ID);
    return result;
}
