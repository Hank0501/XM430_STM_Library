/**
 * @file  my_dynamixel.h
 * @author Z.H. Wu
 * @brief Header file of STM32 library for Dynamixel XM430 servo
 *
 */

#ifndef MY_DYNAMIXEL_H
#define MY_DYNAMIXEL_H

// value for different instruction type
#define INSTRUCTION_PING 1
#define INSTRUCTION_READ 2
#define INSTRUCTION_WRITE 3
#define INSTRUCTION_FactoryReset 6
#define FactoryResetAll 0xff
#define FactoryResetAll_exc_Id 0x01
#define FactoryResetAll_exc_Id_Baud 0x02
#define INSTRUCTION_SYNC_READ 0x82
#define INSTRUCTION_SYNC_WRITE 0x83

// maimum number of parameter used
#define SERVO_MAX_PARAMS 10
#define SERVO_MAX_TxBYTE 100 //  CRC bytes excluded , 8(Fixed bytes, Header ~ Inst) + 4 + (1 + SERVO_MAX_ADDR_SIZE) * SERVO_MAX_COUNT
#define SERVO_MAX_RxBYTE 25  //  The entire Rxd Buffer size
#define SERVO_MAX_COUNT 15
#define SERVO_MAX_ADDR_SIZE 4

// used for response packet prossessing
#define INDEX_SATUS_PACKET_ID 4
#define INDEX_SATUS_PACKET_LEN 5
#define INDEX_SATUS_PACKET_ERR 8

// the basic size of status packet
#define SIZE_STATUS_PACKET 11

// servo starte
#define SERVO_ONLINE 1
#define SERVO_OFFLINE 0xff

//-----------------------------------------------------------------------//
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "control_table.h"

typedef struct ServoResponse
{
    uint8_t RxBuffer[SERVO_MAX_RxBYTE];
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t params[SERVO_MAX_PARAMS];
    uint8_t crc[2];
    bool RxFinished;
} ServoResponse;

typedef struct ServoXM4340
{
    // Servo Control Table
    // int MaxPos;
    // int MinPos;
    // int MaxCurrent;

    int state;

    int BaudRate;
    uint16_t GoalCurrent;
    uint16_t CurrentLimit;
    int32_t Position;
    int16_t Current;
    uint8_t ID;
    uint8_t ReturnDelay;
    uint8_t TorqueENA;
    uint8_t OperatingMode;
    uint8_t DriveMode;

    // Communication used
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *ctrlPort;
    uint16_t ctrlPin;
    volatile ServoResponse Response;

} ServoXM4340;

void setServoResponse_RxFinished(ServoXM4340 *servo, bool val);

uint8_t getRespRxBuffer_ID(ServoXM4340 *servo);

void servo_FactoryReset(ServoXM4340 *servo, uint8_t packetID, uint8_t resetvalue);

void setServo_BaudRate(ServoXM4340 *servo, uint8_t baud);

void setServo_ID(ServoXM4340 *servo, uint8_t id);

void setServo_OperatingMode(ServoXM4340 *servo, uint8_t operatingMode);

void setServo_TorqueENA(ServoXM4340 *servo, uint8_t torque);

void setServo_GoalCurrent(ServoXM4340 *servo, uint16_t current);

void setServo_GoalPosition(ServoXM4340 *servo, const float angle);

void setServo_CurrentLimit(ServoXM4340 *servo, uint16_t current);

void setServo_ProfileAcceleration(ServoXM4340 *servo, uint16_t maxAcc);

void setServo_ProfileVelocity(ServoXM4340 *servo, uint16_t maxVel);

void setServo_DriveMode(ServoXM4340 *servo, uint8_t conf);

HAL_StatusTypeDef setServo_SyncWrite(ServoXM4340 *servoArray, int motorCount, uint8_t addrSize, uint32_t *dataArray, uint32_t addrLB, uint32_t addrHB);

int getServo_BaudRate(ServoXM4340 *servo);

uint8_t getServo_OperatingMode(ServoXM4340 *servo);

uint8_t getServo_TorqueENA(ServoXM4340 *servo);

int16_t getServo_PresentCurrent(ServoXM4340 *servo);

int16_t getServo_GoalCurrent(ServoXM4340 *servo);

uint16_t getServo_CurrentLimit(ServoXM4340 *servo);

int32_t getServo_PresentPosition(ServoXM4340 *servo);

uint16_t getServo_ProfileAcceleration(ServoXM4340 *servo);

uint16_t getServo_ProfileVelocity(ServoXM4340 *servo);

uint8_t getServo_DriveMode(ServoXM4340 *servo);

// void getServo_SyncRead();
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// these shouldn't need to be called externally:
///
void dualTransferServo(ServoXM4340 *servo, int instructionType, int packet_size, uint8_t *params_arr, int param_size);
uint16_t sendServoCommand(UART_HandleTypeDef *huart, uint8_t servoId, uint8_t commandByte, uint8_t numParams, uint8_t *params);

void getServoResponse(ServoXM4340 *servo, uint16_t RxLen);

void clear_RX_buffer(ServoXM4340 *servo);

bool allTrue(bool arr[], int len);

bool checkServoResponse(ServoXM4340 *servo);

unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

void disable_all_IT();

void enable_all_IT();
#endif
