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

// servo state
#define SERVO_ONLINE 1
#define SERVO_OFFLINE 0xff
#define SERVO_SHUTDOWN -1

#define DXL_RX_ERROR_NONE 0
#define DXL_RX_ERROR_TIMEOUT 1
#define DXL_RX_ERROR_INSTRUCTION_FAILED 2
#define DXL_RX_ERROR_CRC_WRONG 3
#define DXL_RX_ERROR_ID_WRONG 4
#define DXL_RX_ERROR_HEADER_WRONG 5

//-----------------------------------------------------------------------//
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "control_table.h"

extern volatile uint8_t DXL_RxBuffer[SERVO_MAX_RX_BUFFER_SIZE * SERVO_MAX_COUNT];
extern volatile uint8_t DXL_TxBuffer[SERVO_MAX_TX_BUFFER_SIZE];
extern volatile bool TxFinished;
extern volatile bool RxFinished;

typedef struct ServoResponse
{
    uint8_t RxBuffer[SERVO_MAX_RX_BUFFER_SIZE];
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t params[SERVO_MAX_PARAMS];
    uint8_t crc[2];
} ServoResponse;

typedef struct ServoXM4340
{
    int state;
    int RX_error;
    int RX_err_ctr;

    int BaudRate;
    uint8_t ID;
    uint8_t OperatingMode;
    uint8_t DriveMode;
    uint8_t ReturnDelay;
    uint8_t Shutdown;
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

    uint16_t Position_PGain;

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

void DXL_InitServo(volatile ServoXM4340 *servo, uint8_t ID, UART_HandleTypeDef *huart, GPIO_TypeDef *ctrlPort, uint16_t ctrlPin);

// void DXL_SetServoResponse_RxFinished(volatile ServoXM4340 *servoList, int servoCount, bool val);

void DXL_SetRxFinished(bool val);

void DXL_SetTxFinished(bool val);

void DXL_AssignRxBufferToServo(ServoXM4340 *servoList, int servoCount, int dataLen);

// uint8_t DXL_GetRxBufferID(void);

/*=================================================================================================*/
/*=================================================================================================*/
/*==================================== Function of factory reset ==================================*/
/*=================================================================================================*/
/*=================================================================================================*/

void servo_FactoryReset(volatile ServoXM4340 *servo, uint8_t packetID, uint8_t resetvalue);

/*=================================================================================================*/
/*=================================================================================================*/
/*============================== Function to write single Servo parameter =========================*/
/*=================================================================================================*/
/*=================================================================================================*/

void setServo_BaudRate(volatile ServoXM4340 *servo, uint8_t baud);

void setServo_ID(volatile ServoXM4340 *servo, uint8_t id);

bool setServo_OperatingMode(volatile ServoXM4340 *servo, uint8_t operatingMode, uint32_t timeout);

bool setServo_TorqueENA(volatile ServoXM4340 *servo, uint8_t torque, uint32_t timeout);

bool setServo_GoalCurrent(volatile ServoXM4340 *servo, float current, uint32_t timeout);

bool setServo_GoalPosition(volatile ServoXM4340 *servo, float angle, uint32_t timeout);

bool setServo_GoalVelocity(volatile ServoXM4340 *servo, float velocity, uint32_t timeout);

bool setServo_CurrentLimit(volatile ServoXM4340 *servo, float current, uint32_t timeout);

bool setServo_VelocityLimit(volatile ServoXM4340 *servo, float velocity, uint32_t timeout);

bool setServo_ProfileAcceleration(volatile ServoXM4340 *servo, uint16_t maxAcc, uint32_t timeout);

bool setServo_ProfileVelocity(volatile ServoXM4340 *servo, uint16_t maxVel, uint32_t timeout);

bool setServo_DriveMode(volatile ServoXM4340 *servo, uint8_t conf, uint32_t timeout);

bool setServo_ReturnDelayTime(volatile ServoXM4340 *servo, uint8_t delay_val, uint32_t timeout);

bool setServo_Position_PGain(volatile ServoXM4340 *servo, uint16_t p_gain, uint32_t timeout);

/*=================================================================================================*/
/*=================================================================================================*/
/*===========================  Function to Read single Servo parameter  ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

bool getServo_Shutdown(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_BaudRate(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_OperatingMode(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_TorqueENA(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_PresentCurrent(volatile ServoXM4340 *servo, uint32_t timeout, float *outCurrent);

bool getServo_GoalCurrent(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_CurrentLimit(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_PresentPosition(volatile ServoXM4340 *servo, uint32_t timeout, float *outPosition);

bool getServo_PresentVelocity(volatile ServoXM4340 *servo, uint32_t timeout, float *outVelocity);

bool getServo_GoalVelocity(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_VelocityLimit(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_ProfileAcceleration(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_ProfileVelocity(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_DriveMode(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_ReturnDelayTime(volatile ServoXM4340 *servo, uint32_t timeout);

bool getServo_Position_PGain(volatile ServoXM4340 *servo, uint32_t timeout);

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function to "Syncwrite" multiple Servo parameter ==========================*/
/*=================================================================================================*/
/*=================================================================================================*/

void syncWrite_GoalPosition(volatile ServoXM4340 *servoList, int servoCount, const float *angleList);

void syncWrite_GoalCurrent(volatile ServoXM4340 *servoList, int servoCount, const float *currentList);

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function to "Syncread" multiple Servo parameter ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

bool syncRead_ID(volatile ServoXM4340 *servoList, int servoCount, uint32_t timeout);

bool syncRead_PresentPosition(volatile ServoXM4340 *servoList, int servoCount, float *posList, uint32_t timeout);

bool syncRead_PresentCurrent(volatile ServoXM4340 *servoList, int servoCount, float *curList, uint32_t timeout);

bool syncRead_PresentVelocity(volatile ServoXM4340 *servoList, int servoCount, float *velList, uint32_t timeout);

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function that should not  be called externally: ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

void setServo_SyncWrite(volatile ServoXM4340 *servoArray, int servoCount, uint8_t addrLB, uint8_t addrHB, uint8_t addrSize, uint32_t *dataArray);

bool getServo_SyncRead(volatile ServoXM4340 *servoArray, int servoCount, uint8_t addrLB, uint8_t addrHB, uint8_t addrSize, uint32_t timeout);

bool dualTransferServo(volatile ServoXM4340 *servo, int instructionType, int packet_size, uint8_t *params_arr, int param_size, uint32_t timeout);

uint16_t sendServoCommand(UART_HandleTypeDef *huart, uint8_t servoId, uint8_t commandByte, uint8_t numParams, uint8_t *params);

void getServoResponse(volatile ServoXM4340 *servo, uint16_t RxLen, uint32_t timeout);

void clear_Servo_RX_buffer(volatile ServoXM4340 *servo);

void clear_RX_buffer(void);

void clear_TX_buffer(void);

// bool allTrue(int state, int len);

bool checkServoResponse(volatile ServoXM4340 *servo);

unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

void disable_all_IT();

void enable_all_IT();
#endif
