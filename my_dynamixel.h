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
#define SERVO_MAX_TX_BUFFER_SIZE 100 //  CRC bytes excluded , 8(Fixed bytes, Header ~ Inst) + 4 + (1 + SERVO_MAX_ADDR_SIZE) * SERVO_MAX_COUNT
#define SERVO_MAX_RX_BUFFER_SIZE 25  //  The entire Rxd Buffer size
#define SERVO_MAX_COUNT 15
#define SERVO_MAX_ADDR_SIZE 4

// used for response packet prossessing
#define INDEX_SATUS_PACKET_ID 4
#define INDEX_SATUS_PACKET_LEN_L 5
#define INDEX_SATUS_PACKET_LEN_H 6
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

extern volatile uint8_t DXL_RxBuffer[SERVO_MAX_RX_BUFFER_SIZE];
extern volatile uint8_t DXL_TxBuffer[SERVO_MAX_TX_BUFFER_SIZE];
extern volatile bool TxFinished;

typedef struct ServoResponse
{
    uint8_t RxBuffer[SERVO_MAX_RX_BUFFER_SIZE];
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t params[SERVO_MAX_PARAMS];
    uint8_t crc[2];
    bool RxFinished;
} ServoResponse;

typedef struct ServoXM4340
{
    int state;

    int BaudRate;
    uint8_t ID;
    uint8_t OperatingMode;
    uint8_t DriveMode;
    uint8_t ReturnDelay;
    uint8_t TorqueENA;

    int32_t PresentPosition;

    int16_t PresentCurrent;
    int16_t GoalCurrent;
    uint16_t CurrentLimit;

    int32_t PresentVelocity;
    int32_t GoalVelocity;
    uint32_t VelocityLimit;

    uint32_t ProfileAcceleration;
    uint32_t ProfileVelocity;

    // Communication used
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *ctrlPort;
    uint16_t ctrlPin;
    volatile ServoResponse Response;

} ServoXM4340;

/*=================================================================================================*/
/*=================================================================================================*/
/*======================== Function that user can reach Servo structure ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

void DXL_SetServoResponse_RxFinished(ServoXM4340 *servo, bool val);

void DXL_SetTxFinished(bool val);

void DXL_AssignRxBufferToServo(ServoXM4340 *servo);

uint8_t DXL_GetRxBufferID(void);

/*=================================================================================================*/
/*=================================================================================================*/
/*==================================== Function of factory reset ==================================*/
/*=================================================================================================*/
/*=================================================================================================*/

void servo_FactoryReset(ServoXM4340 *servo, uint8_t packetID, uint8_t resetvalue);

/*=================================================================================================*/
/*=================================================================================================*/
/*============================== Function to write single Servo parameter =========================*/
/*=================================================================================================*/
/*=================================================================================================*/

void setServo_BaudRate(ServoXM4340 *servo, uint8_t baud);

void setServo_ID(ServoXM4340 *servo, uint8_t id);

void setServo_OperatingMode(ServoXM4340 *servo, uint8_t operatingMode);

void setServo_TorqueENA(ServoXM4340 *servo, uint8_t torque);

void setServo_GoalCurrent(ServoXM4340 *servo, float current);

void setServo_GoalPosition(ServoXM4340 *servo, float angle);

void setServo_GoalVelocity(ServoXM4340 *servo, float velocity);

void setServo_CurrentLimit(ServoXM4340 *servo, float current);

void setServo_VelocityLimit(ServoXM4340 *servo, float velocity);

void setServo_ProfileAcceleration(ServoXM4340 *servo, uint16_t maxAcc);

void setServo_ProfileVelocity(ServoXM4340 *servo, uint16_t maxVel);

void setServo_DriveMode(ServoXM4340 *servo, uint8_t conf);

/*=================================================================================================*/
/*=================================================================================================*/
/*===========================  Function to Read single Servo parameter  ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

void getServo_BaudRate(ServoXM4340 *servo);

void getServo_OperatingMode(ServoXM4340 *servo);

void getServo_TorqueENA(ServoXM4340 *servo);

float getServo_PresentCurrent(ServoXM4340 *servo);

void getServo_GoalCurrent(ServoXM4340 *servo);

void getServo_CurrentLimit(ServoXM4340 *servo);

float getServo_PresentPosition(ServoXM4340 *servo);

float getServo_PresentVelocity(ServoXM4340 *servo);

void getServo_GoalVelocity(ServoXM4340 *servo);

void getServo_VelocityLimit(ServoXM4340 *servo);

void getServo_ProfileAcceleration(ServoXM4340 *servo);

void getServo_ProfileVelocity(ServoXM4340 *servo);

void getServo_DriveMode(ServoXM4340 *servo);

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function to "Syncwrite" multiple Servo parameter ==========================*/
/*=================================================================================================*/
/*=================================================================================================*/

void syncWrite_GoalPosition(ServoXM4340 *servoList, int servoCount, const float *angleList);

void syncWrite_GoalCurrent(ServoXM4340 *servoList, int servoCount, const int16_t *currentList);

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function to "Syncread" multiple Servo parameter ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

void syncRead_PresentPosition(ServoXM4340 *servoList, int servoCount, float *posList);

void syncRead_PresentCurrent(ServoXM4340 *servoList, int servoCount, float *curList);

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function that should not  be called externally: ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

void setServo_SyncWrite(ServoXM4340 *servoArray, int servoCount, uint8_t addrLB, uint8_t addrHB, uint8_t addrSize, uint32_t *dataArray);

HAL_StatusTypeDef getServo_SyncRead(ServoXM4340 *servoArray, int servoCount, uint8_t addrLB, uint8_t addrHB, uint8_t addrSize);

void dualTransferServo(ServoXM4340 *servo, int instructionType, int packet_size, uint8_t *params_arr, int param_size);

uint16_t sendServoCommand(UART_HandleTypeDef *huart, uint8_t servoId, uint8_t commandByte, uint8_t numParams, uint8_t *params);

void getServoResponse(ServoXM4340 *servo, uint16_t RxLen);

void clear_RX_buffer(ServoXM4340 *servo);

void clear_TX_buffer(void);

bool allTrue(int arr[], int len);

bool checkServoResponse(ServoXM4340 *servo);

unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

void disable_all_IT();

void enable_all_IT();
#endif
