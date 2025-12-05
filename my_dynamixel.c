/**
 * @file  my_dynamixel.c
 * @author  Z.H. Wu
 * @brief  STM32 library for Dynamixel XM430 servo
 *
 * @note  Most of the function work based on knowing Servo ID,
 *          please use servo_FactoryReset and getServoResponse_ID to reset servo when you dont know the Servo information.
 *        Sync Read/Write cannot be used when there are more than 256 motors.
 */

#include "my_dynamixel.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>

volatile uint8_t DXL_RxBuffer[SERVO_MAX_RX_BUFFER_SIZE * SERVO_MAX_COUNT];
volatile uint8_t DXL_TxBuffer[SERVO_MAX_TX_BUFFER_SIZE];
volatile bool TxFinished;
volatile bool RxFinished;

/*=================================================================================================*/
/*=================================================================================================*/
/*======================== Function that user can reach Servo structure ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief  Init the servo structure param.
 * @param  servo ServoXM430 structure
 * @param  ID Servo ID
 * @param  huart Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @param  ctrlPort GPIOx, x = A, B,...
 * @param  ctrlPin GPIO_PIN_x, x = 1, 2,...
 * @retval  None
 */
void DXL_InitServo(volatile ServoXM4340 *servo, uint8_t ID, UART_HandleTypeDef *huart, GPIO_TypeDef *ctrlPort, uint16_t ctrlPin)
{
    servo->ID = ID;
    servo->huart = huart;
    servo->ctrlPort = ctrlPort;
    servo->ctrlPin = ctrlPin;
}

void DXL_SetRxFinished(bool val)
{
    RxFinished = val;
}

void DXL_SetTxFinished(bool val)
{
    TxFinished = val;
}

void DXL_AssignRxBufferToServo(ServoXM4340 *servoList, int servoCount, int dataLen)
{
    uint16_t index = 0;

    while (index + 7 <= dataLen) // 7 = header + len
    {
        // check header
        if (DXL_RxBuffer[index] == 0xFF &&
            DXL_RxBuffer[index + 1] == 0xFF &&
            DXL_RxBuffer[index + 2] == 0xFD &&
            DXL_RxBuffer[index + 3] == 0x00)
        {
            uint8_t id = DXL_RxBuffer[index + 4];
            uint16_t packetLen = ((uint16_t)DXL_RxBuffer[index + 5] | (DXL_RxBuffer[index + 6] << 8)) + 7;

            if (id != ID_broadcast)
            {
                for (uint8_t i = 0; i < servoCount; i++)
                {
                    // assign to the corresponded servo
                    if (servoList[i].ID == id)
                    {
                        if (packetLen <= SERVO_MAX_RX_BUFFER_SIZE)
                        {
                            memcpy(servoList[i].Response.RxBuffer, &DXL_RxBuffer[index], packetLen);
                        }
                        break;
                    }
                }
            }
            else
            {
                // BROADCAST_ID case
            }

            // move to next packet
            index += packetLen;
        }
        else
            break;
    }
    // clear_DXL_RX_buffer(dataLen);
}

/*=================================================================================================*/
/*=================================================================================================*/
/*==================================== Function of factory reset ==================================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief  Reset Servo
 * @param  servo ServoXM430 structure
 * @param  resetvalue
 *      This parameter can be one of the following values:
 *      @arg  FactoryResetAll: Reset all
 *      @arg  FactoryResetAll_exc_Id:  Reset all except ID
 *      @arg  FactoryResetAll_exc_Id_Baud: Reset all except ID and Baudrate
 * @retval  None
 * @note  When the Packet ID is the Broadcast ID 0xFE and the configured Option is Reset All,
 *        the Factory Reset Instruction(0x06) will NOT be executed.
 */
void servo_FactoryReset(volatile ServoXM4340 *servo, uint8_t packetID, uint8_t resetvalue)
{

    // parameters calculated and send the instruction
    uint8_t params_arr[1] = {0};
    params_arr[0] = resetvalue;

    // RxBuffer clear
    clear_Servo_RX_buffer(servo);

    // TX turn on
    HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_SET);

    sendServoCommand(servo->huart, packetID, INSTRUCTION_FactoryReset, 1, params_arr);

    while (!TxFinished)
    {
        // wait until transmitting finished;
    };

    // // RX turn on
    // HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_RESET);

    // HAL_UART_DeInit(servo->huart);
    // (*(servo->huart)).Init.BaudRate = 57600;
    // if (HAL_UART_Init(servo->huart) != HAL_OK)
    // {
    // }

    // // The DYNAMIXEL Broadcast ID (254 (0xFE)) will only return Status Packets for Ping, Sync Read and Bulk Read commands
    // if (packetID != ID_broadcast)
    // {
    //     getServoResponse(servo, SIZE_STATUS_PACKET);
    //     checkServoResponse(servo);
    // }
    // else
    // {
    //     HAL_UART_Receive(servo->huart, servo->Response.RxBuffer, SIZE_STATUS_PACKET, 0xffff);
    // }
}

/*=================================================================================================*/
/*=================================================================================================*/
/*============================== Function to write single Servo parameter =========================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief  Set the servo BaudRate parameter.
 * @param  servo ServoXM430 structure
 * @param  baud Baudrate value
 *      This parameter can be one of the following values:
 *      @arg  BaudRate_57600: Baud Rate 57600 Mbps
 *      @arg  BaudRate_115200: Baud Rate 115200 Mbps
 *      @arg  BaudRate_1M: Baud Rate 1M Mbps
 *      @arg  BaudRate_2M: Baud Rate 2M Mbps
 *      @arg  BaudRate_3M: Baud Rate 3M Mbps
 *      @arg  BaudRate_4M: Baud Rate 4M Mbps
 *      @arg  BaudRate_4p5M: Baud Rate 4.5M Mbps
 * @retval  None
 * @note  The servo return status packet in original baudrate
 */
void setServo_BaudRate(volatile ServoXM4340 *servo, uint8_t baud)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[3] = {0};
    params_arr[0] = BaudRate_ADDR_LB;
    params_arr[1] = BaudRate_ADDR_HB;
    params_arr[2] = baud;

    // RxBuffer clear
    clear_Servo_RX_buffer(servo);

    // DXL_RxBuffer ready
    HAL_UART_Receive_DMA(servo->huart, DXL_RxBuffer, SIZE_STATUS_PACKET);

    // TX turn on
    HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_SET);

    sendServoCommand(servo->huart, servo->ID, INSTRUCTION_WRITE, 3, params_arr);

    while (!TxFinished)
    {
        // wait until transmitting finished;
    };

    // RX turn on
    HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_RESET);

    getServoResponse(servo, SIZE_STATUS_PACKET);
    checkServoResponse(servo);
}

/**
 * @brief  Set the servo ID parameter.
 * @param  servo Pointer to ServoXM430 structure
 * @param  id New ID
 * @retval  None
 * @note  The servo return status packet in original ID value
 */
void setServo_ID(volatile ServoXM4340 *servo, uint8_t id)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[3] = {0};
    params_arr[0] = ID_ADDR_LB;
    params_arr[1] = ID_ADDR_HB;
    params_arr[2] = id;

    // RxBuffer clear
    clear_Servo_RX_buffer(servo);

    // DXL_RxBuffer ready
    HAL_UART_Receive_DMA(servo->huart, DXL_RxBuffer, SIZE_STATUS_PACKET);

    // TX turn on
    HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_SET);

    sendServoCommand(servo->huart, servo->ID, INSTRUCTION_WRITE, 3, params_arr);

    while (!TxFinished)
    {
        // wait until transmitting finished;
    };

    // RX turn on
    HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_RESET);

    getServoResponse(servo, SIZE_STATUS_PACKET);
}

/**
 * @brief  Set the servo Operating Mode parameter.
 * @param  servo ServoXM430 structure
 * @param  operatingMode Servo operating mode
 *      This parameter can be one of the following values:
 *      @arg  Current_CtrlMode: Current control mode
 *      @arg  Velocity_CtrlMode: Velocity control mode
 *      @arg  POS_CtrlMode: Position control mode
 *      @arg  Extended_Pos_CtrlMode: Extended Position control mode
 *      @arg  PWM_CtrlMode: PWm control mode
 * @retval  None
 */
void setServo_OperatingMode(volatile ServoXM4340 *servo, uint8_t operatingMode)
{

    // parameters calculated and send the instruction
    uint8_t params_arr[3] = {0};
    params_arr[0] = OperatingMode_ADDR_LB;
    params_arr[1] = OperatingMode_ADDR_HB;
    params_arr[2] = operatingMode;

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

/**
 * @brief  Set the servo TorqueEnable parameter.
 * @param  servo ServoXM430 structure
 * @param  torque torque on or off
 *      This parameter can be one of the following values:
 *      @arg  TORQUE_ENABLE: Torque enable
 *      @arg  TORQUE_DISABLE: Torque disable
 * @retval  None
 */
void setServo_TorqueENA(volatile ServoXM4340 *servo, uint8_t torque)
{

    // parameters calculated and send the instruction
    uint8_t params_arr[3] = {0};
    params_arr[0] = TorqueEnable_ADDR_LB;
    params_arr[1] = TorqueEnable_ADDR_HB;
    params_arr[2] = torque;

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

/**
 * @brief  Set the servo Goal Current parameter.
 * @param  servo ServoXM430 structure
 * @param  current Goal current value in mA
 * @retval  None
 * @note  Goal current value should not exceed Current Limit value
 */
void setServo_GoalCurrent(volatile ServoXM4340 *servo, float current)
{

    // parameters calculated and send the instruction
    int16_t cur = roundf(current / DXL_CUR_RESOLUTION);
    uint8_t params_arr[4] = {0};
    params_arr[0] = GoalCurrent_ADDR_LB;
    params_arr[1] = GoalCurrent_ADDR_HB;
    params_arr[2] = (uint8_t)cur & 0x00ff;
    params_arr[3] = (uint8_t)(cur >> 8) & (0x00ff);

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

/**
 * @brief  Set the servo Goal position parameter.
 * @param  servo ServoXM430 structure
 * @param  angle Goal position value, in unit of degree
 * @retval  None
 * @note  Goal position value has limitation in different operrating mode, see manual.
 */
void setServo_GoalPosition(volatile ServoXM4340 *servo, float angle)
{
    // parameters calculated and send the instruction
    int32_t ang = roundf(angle / DXL_POS_RESOLUTION);
    uint8_t params_arr[6] = {0};
    params_arr[0] = GoalPosition_ADDR_LB;
    params_arr[1] = GoalPosition_ADDR_HB;
    params_arr[2] = (uint8_t)ang & 0x000000ff;
    params_arr[3] = (uint8_t)(ang >> 8) & (0x000000ff);
    params_arr[4] = (uint8_t)(ang >> 16) & (0x000000ff);
    params_arr[5] = (uint8_t)(ang >> 24) & (0x000000ff);

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

void setServo_GoalVelocity(volatile ServoXM4340 *servo, float velocity)
{

    // parameters calculated and send the instructions
    int32_t vel = roundf(velocity / DXL_VEL_RESOLUTION);
    uint8_t params_arr[6] = {0};
    params_arr[0] = GoalVelocity_ADDR_LB;
    params_arr[1] = GoalVelocity_ADDR_HB;
    params_arr[2] = (uint8_t)vel & 0x000000ff;
    params_arr[3] = (uint8_t)(vel >> 8) & (0x000000ff);
    params_arr[4] = (uint8_t)(vel >> 16) & (0x000000ff);
    params_arr[5] = (uint8_t)(vel >> 24) & (0x000000ff);

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

/**
 * @brief  Set the servo Current Limit parameter.
 * @param  servo ServoXM430 structure
 * @param  current Current limit value
 * @retval  None
 * @note  Range of current limit value is 0~1193 for XM430
 */
void setServo_CurrentLimit(volatile ServoXM4340 *servo, float current)
{
    // parameters calculated and send the instruction
    int16_t cur = roundf(current / DXL_CUR_RESOLUTION);
    uint8_t params_arr[4] = {0};
    params_arr[0] = CurrentLimit_ADDR_LB;
    params_arr[1] = CurrentLimit_ADDR_HB;
    params_arr[2] = (uint8_t)cur & 0x00ff;
    params_arr[3] = (uint8_t)(cur >> 8) & (0x00ff);

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

void setServo_VelocityLimit(volatile ServoXM4340 *servo, float velocity)
{
    // parameters calculated and send the instruction
    int32_t vel = roundf(velocity / DXL_VEL_RESOLUTION);
    uint8_t params_arr[6] = {0};
    params_arr[0] = VelocityLimit_ADDR_LB;
    params_arr[1] = VelocityLimit_ADDR_HB;
    params_arr[2] = (uint8_t)vel & 0x000000ff;
    params_arr[3] = (uint8_t)(vel >> 8) & (0x000000ff);
    params_arr[4] = (uint8_t)(vel >> 16) & (0x000000ff);
    params_arr[5] = (uint8_t)(vel >> 24) & (0x000000ff);

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

void setServo_ProfileAcceleration(volatile ServoXM4340 *servo, uint16_t maxAcc)
{
    // parameters calculated and send the instruction

    uint8_t params_arr[6] = {0};
    params_arr[0] = ProfileAcceleration_ADDR_LB;
    params_arr[1] = ProfileAcceleration_ADDR_HB;
    if ((servo->DriveMode & (1 << 2)) && maxAcc > ProfileVelocityLimit_T_Based)
    {
        // Time-based Profile
        params_arr[2] = (uint8_t)ProfileAccelerationLimit_T_Based & 0x00ff;
        params_arr[3] = (uint8_t)(ProfileAccelerationLimit_T_Based >> 8) & (0x00ff);
        params_arr[4] = (uint8_t)(ProfileAccelerationLimit_T_Based >> 16) & (0x00ff);
        params_arr[5] = (uint8_t)(ProfileAccelerationLimit_T_Based >> 24) & (0x00ff);
    }
    else if ((servo->DriveMode & (1 << 2)) == 0 && maxAcc > ProfileAccelerationLimit_V_Based)
    {
        // Velocity-based Profile
        params_arr[2] = (uint8_t)ProfileAccelerationLimit_V_Based & 0x00ff;
        params_arr[3] = (uint8_t)(ProfileAccelerationLimit_V_Based >> 8) & (0x00ff);
        params_arr[4] = (uint8_t)(ProfileAccelerationLimit_V_Based >> 16) & (0x00ff);
        params_arr[5] = (uint8_t)(ProfileAccelerationLimit_V_Based >> 24) & (0x00ff);
    }
    else
    {
        params_arr[2] = (uint8_t)maxAcc & 0x00ff;
        params_arr[3] = (uint8_t)(maxAcc >> 8) & (0x00ff);
        params_arr[4] = (uint8_t)(maxAcc >> 16) & (0x00ff);
        params_arr[5] = (uint8_t)(maxAcc >> 24) & (0x00ff);
    }

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

void setServo_ProfileVelocity(volatile ServoXM4340 *servo, uint16_t maxVel)
{

    // parameters calculated and send the instruction

    uint8_t params_arr[6] = {0};
    params_arr[0] = ProfileVelocity_ADDR_LB;
    params_arr[1] = ProfileVelocity_ADDR_HB;
    if ((servo->DriveMode & (1 << 2)) && maxVel > ProfileVelocityLimit_T_Based)
    {
        // Time-based Profile
        params_arr[2] = (uint8_t)ProfileVelocityLimit_T_Based & 0x00ff;
        params_arr[3] = (uint8_t)(ProfileVelocityLimit_T_Based >> 8) & (0x00ff);
        params_arr[4] = (uint8_t)(ProfileVelocityLimit_T_Based >> 16) & (0x00ff);
        params_arr[5] = (uint8_t)(ProfileVelocityLimit_T_Based >> 24) & (0x00ff);
    }
    else if ((servo->DriveMode & (1 << 2)) == 0 && maxVel > ProfileVelocityLimit_V_Based)
    {
        // Velocity-based Profile
        params_arr[2] = (uint8_t)ProfileVelocityLimit_V_Based & 0x00ff;
        params_arr[3] = (uint8_t)(ProfileVelocityLimit_V_Based >> 8) & (0x00ff);
        params_arr[4] = (uint8_t)(ProfileVelocityLimit_V_Based >> 16) & (0x00ff);
        params_arr[5] = (uint8_t)(ProfileVelocityLimit_V_Based >> 24) & (0x00ff);
    }
    else
    {
        params_arr[2] = (uint8_t)maxVel & 0x00ff;
        params_arr[3] = (uint8_t)(maxVel >> 8) & (0x00ff);
        params_arr[4] = (uint8_t)(maxVel >> 16) & (0x00ff);
        params_arr[5] = (uint8_t)(maxVel >> 24) & (0x00ff);
    }

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

void setServo_DriveMode(volatile ServoXM4340 *servo, uint8_t conf)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[3] = {0};
    params_arr[0] = DriveMode_ADDR_LB;
    params_arr[1] = DriveMode_ADDR_HB;
    params_arr[2] = conf;

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

/**
 * @brief  Set the servo Current Limit parameter.
 * @param  servo ServoXM430 structure
 * @param  delay_val Return delay time in the unit of 2[μsec]
 * @retval  None
 * @note  Range of delay_val is 0 ~ 254 for XM430
 */
void setServo_ReturnDelayTime(volatile ServoXM4340 *servo, uint8_t delay_val)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[3] = {0};
    params_arr[0] = ReturnDelayTime_ADDR_LB;
    params_arr[1] = ReturnDelayTime_ADDR_HB;
    params_arr[2] = delay_val;

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

void setServo_Position_PGain(volatile ServoXM4340 *servo, uint16_t p_gain)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = PositionPGain_ADDR_LB;
    params_arr[1] = PositionPGain_ADDR_HB;
    params_arr[2] = (uint8_t)p_gain & 0x00ff;
    params_arr[3] = (uint8_t)(p_gain >> 8) & (0x00ff);

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

/*=================================================================================================*/
/*=================================================================================================*/
/*===========================  Function to Read single Servo parameter  ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief Get the servo Baudrate value, value of Baudrate variable in "servo" structure will be modified.
 * @param servo ServoXM430 structure
 * @retval None
 */
void getServo_BaudRate(volatile ServoXM4340 *servo)
{

    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = BaudRate_ADDR_LB;
    params_arr[1] = BaudRate_ADDR_HB;
    params_arr[2] = BaudRate_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + BaudRate_ByteSize, params_arr, sizeof(params_arr));

    switch (servo->Response.params[0])
    {
    case BaudRate_57600:
        servo->BaudRate = 57600;
        break;
    case BaudRate_115200:
        servo->BaudRate = 115200;
        break;
    case BaudRate_1M:
        servo->BaudRate = 1000000;
        break;
    case BaudRate_2M:
        servo->BaudRate = 2000000;
        break;
    case BaudRate_3M:
        servo->BaudRate = 3000000;
        break;
    case BaudRate_4M:
        servo->BaudRate = 4000000;
        break;
    case BaudRate_4p5M:
        servo->BaudRate = 4500000;
        break;
    default:
        break;
    }
}

/**
 * @brief Get the servo TorqueEnable parameter value, value of TorqueEnable variable in "servo" structure will be modified.
 * @param servo ServoXM430 structure
 * @retval None
 */
void getServo_TorqueENA(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = TorqueEnable_ADDR_LB;
    params_arr[1] = TorqueEnable_ADDR_HB;
    params_arr[2] = TorqueEnable_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + TorqueEnable_ByteSize, params_arr, sizeof(params_arr));

    servo->TorqueENA = servo->Response.params[0];
}

/**
 * @brief Get the servo PresentCurrent parameter value, value of PresentCurrent variable in "servo" structure will be modified.
 * @param servo ServoXM430 structure
 * @retval Present Current value, in mA
 */
float getServo_PresentCurrent(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = PresentCurrent_ADDR_LB;
    params_arr[1] = PresentCurrent_ADDR_HB;
    params_arr[2] = PresentCurrent_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + PresentCurrent_ByteSize, params_arr, sizeof(params_arr));

    // record the current  of the servo
    int16_t pre_cur = 0;
    pre_cur |= (int16_t)servo->Response.params[0];
    pre_cur |= (int16_t)servo->Response.params[1] << 8;

    servo->PresentCurrent = pre_cur;

    return (float)pre_cur * DXL_CUR_RESOLUTION;
}

/**
 * @brief Get the servo GoalCurrent parameter value, value of GoalCurrent variable in "servo" structure will be modified.
 * @param servo ServoXM430 structure
 * @retval None
 *
 * @note GoalCurrent value reset to CurrentLimit after CurrentLimit parameter is set.
 */
void getServo_GoalCurrent(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = GoalCurrent_ADDR_LB;
    params_arr[1] = GoalCurrent_ADDR_HB;
    params_arr[2] = GoalCurrent_ByteSize;
    params_arr[3] = 0x00;
    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + GoalCurrent_ByteSize, params_arr, sizeof(params_arr));

    // record the current  of the servo
    int16_t goal_cur = 0;
    goal_cur |= (int16_t)servo->Response.params[0];
    goal_cur |= (int16_t)servo->Response.params[1] << 8;

    servo->GoalCurrent = goal_cur;
}

/**
 * @brief  Get the servo CurrentLimit parameter value, value of CurrentLimit variable in "servo" structure will be modified.
 * @param  servo ServoXM430 structure
 * @retval  None
 */
void getServo_CurrentLimit(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = CurrentLimit_ADDR_LB;
    params_arr[1] = CurrentLimit_ADDR_HB;
    params_arr[2] = CurrentLimit_ByteSize;
    params_arr[3] = 0x00;
    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + CurrentLimit_ByteSize, params_arr, sizeof(params_arr));

    // record the current  of the servo
    uint16_t cur = 0;
    cur |= (uint16_t)servo->Response.params[0];
    cur |= (uint16_t)servo->Response.params[1] << 8;
    servo->CurrentLimit = cur;
}

/**
 * @brief  Get the servo PresentPosition value, value of PresentPosition variable in "servo" structure will be modified.
 * @param  servo ServoXM430 structure
 * @retval PresentPosition value in deg unit
 */
float getServo_PresentPosition(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = PresentPosition_ADDR_LB;
    params_arr[1] = PresentPosition_ADDR_HB;
    params_arr[2] = PresentPosition_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + PresentPosition_ByteSize, params_arr, sizeof(params_arr));

    // record the postion of the servo
    int32_t pos = 0;
    pos |= (uint32_t)servo->Response.params[0];
    pos |= (uint32_t)servo->Response.params[1] << 8;
    pos |= (uint32_t)servo->Response.params[2] << 16;
    pos |= (uint32_t)servo->Response.params[3] << 24;

    servo->PresentPosition = pos;

    return (float)pos * DXL_POS_RESOLUTION;
}

float getServo_PresentVelocity(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = PresentVelocity_ADDR_LB;
    params_arr[1] = PresentVelocity_ADDR_HB;
    params_arr[2] = PresentVelocity_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + PresentVelocity_ByteSize, params_arr, sizeof(params_arr));

    // record the velocity of the servo
    int32_t vel = 0;
    vel |= (uint32_t)servo->Response.params[0];
    vel |= (uint32_t)servo->Response.params[1] << 8;
    vel |= (uint32_t)servo->Response.params[2] << 16;
    vel |= (uint32_t)servo->Response.params[3] << 24;

    servo->PresentVelocity = vel;

    return (float)vel * DXL_VEL_RESOLUTION;
}

void getServo_GoalVelocity(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = GoalVelocity_ADDR_LB;
    params_arr[1] = GoalVelocity_ADDR_HB;
    params_arr[2] = GoalVelocity_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + GoalVelocity_ByteSize, params_arr, sizeof(params_arr));

    // record the velocity of the servo
    int32_t vel = 0;
    vel |= (uint32_t)servo->Response.params[0];
    vel |= (uint32_t)servo->Response.params[1] << 8;
    vel |= (uint32_t)servo->Response.params[2] << 16;
    vel |= (uint32_t)servo->Response.params[3] << 24;

    servo->GoalVelocity = vel;
}

void getServo_VelocityLimit(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = VelocityLimit_ADDR_LB;
    params_arr[1] = VelocityLimit_ADDR_HB;
    params_arr[2] = VelocityLimit_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + VelocityLimit_ByteSize, params_arr, sizeof(params_arr));

    // record the velocity of the servo
    uint32_t vel = 0;
    vel |= (uint32_t)servo->Response.params[0];
    vel |= (uint32_t)servo->Response.params[1] << 8;
    vel |= (uint32_t)servo->Response.params[2] << 16;
    vel |= (uint32_t)servo->Response.params[3] << 24;

    servo->VelocityLimit = vel;
}

/**
 * @brief  Get the servo OperatingMode parameter value, value of OperatingMode variable in "servo" structure will be modified.
 * @param  servo ServoXM430 structure
 * @retval  None
 */
void getServo_OperatingMode(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = OperatingMode_ADDR_LB;
    params_arr[1] = OperatingMode_ADDR_HB;
    params_arr[2] = OperatingMode_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + OperatingMode_ByteSize, params_arr, sizeof(params_arr));

    servo->OperatingMode = servo->Response.params[0];
}

void getServo_ProfileAcceleration(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = ProfileAcceleration_ADDR_LB;
    params_arr[1] = ProfileAcceleration_ADDR_HB;
    params_arr[2] = ProfileAcceleration_ByteSize;
    params_arr[3] = 0x00;
    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + ProfileAcceleration_ByteSize, params_arr, sizeof(params_arr));

    // record the ProfileAcceleration  of the servo
    uint32_t prof = 0;
    prof |= (uint32_t)servo->Response.params[0];
    prof |= (uint32_t)servo->Response.params[1] << 8;
    prof |= (uint32_t)servo->Response.params[2] << 16;
    prof |= (uint32_t)servo->Response.params[3] << 24;
    servo->ProfileAcceleration = prof;
}

void getServo_ProfileVelocity(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = ProfileVelocity_ADDR_LB;
    params_arr[1] = ProfileVelocity_ADDR_HB;
    params_arr[2] = ProfileVelocity_ByteSize;
    params_arr[3] = 0x00;
    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + ProfileVelocity_ByteSize, params_arr, sizeof(params_arr));

    // record the ProfileVelocity  of the servo
    uint32_t prof = 0;
    prof |= (uint32_t)servo->Response.params[0];
    prof |= (uint32_t)servo->Response.params[1] << 8;
    prof |= (uint32_t)servo->Response.params[2] << 16;
    prof |= (uint32_t)servo->Response.params[3] << 24;
    servo->ProfileVelocity = prof;
}

void getServo_DriveMode(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = DriveMode_ADDR_LB;
    params_arr[1] = DriveMode_ADDR_HB;
    params_arr[2] = DriveMode_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + DriveMode_ByteSize, params_arr, sizeof(params_arr));
    servo->DriveMode = servo->Response.params[0];
}

void getServo_ReturnDelayTime(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = ReturnDelayTime_ADDR_LB;
    params_arr[1] = ReturnDelayTime_ADDR_HB;
    params_arr[2] = ReturnDelayTime_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + ReturnDelayTime_ByteSize, params_arr, sizeof(params_arr));
    servo->ReturnDelay = servo->Response.params[0];
}

void getServo_Position_PGain(volatile ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = PositionPGain_ADDR_LB;
    params_arr[1] = PositionPGain_ADDR_HB;
    params_arr[2] = PositionPGain_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + PositionPGain_ByteSize, params_arr, sizeof(params_arr));

    uint16_t p_gain = 0;
    p_gain |= (uint16_t)servo->Response.params[0];
    p_gain |= (uint16_t)servo->Response.params[1] << 8;
    servo->Position_PGain = p_gain;
}

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function to "Syncwrite" multiple Servo parameter ==========================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief  Set the servo GoalPosition parameter at the same time.
 * @param  servoList An array of volatile ServoXM4340 structures
 * @param  servoCount Size of servoArray, which represents the number of servo
 * @param  angleList An array of angle command to write into each servo
 * @retval  None
 * @note  Goalposition value has limitation in different operrating mode, see manual.
 * @note  Servos don't return status packet for SyncWrite instruction.
 */
void syncWrite_GoalPosition(volatile ServoXM4340 *servoList, int servoCount, const float *angleList)
{
    uint32_t dataArray[SERVO_MAX_COUNT];
    for (int i = 0; i < servoCount; i++)
    {
        int32_t temp = angleList[i] / DXL_POS_RESOLUTION; // 0.08789 = 360/4096
        uint32_t angle_cmd = temp;                        // in 2's complement
        dataArray[i] = angle_cmd;
    }
    setServo_SyncWrite(servoList, servoCount, GoalPosition_ADDR_LB, GoalPosition_ADDR_HB, GoalPosition_ByteSize, dataArray);
}

/**
 * @brief  Set the servo GoalCurrent parameter at the same time.
 * @param  servoList An array of volatile ServoXM4340 structures
 * @param  servoCount Size of servoArray, which represents the number of servo
 * @param  currentList An array of current command to write into each servo
 * @retval  None
 * @note  GoalCurrent can not be set larger than the CurrentLimit parameter.
 * @note  Servos don't return status packet for SyncWrite instruction.
 */
void syncWrite_GoalCurrent(volatile ServoXM4340 *servoList, int servoCount, const float *currentList)
{
    uint32_t dataArray[SERVO_MAX_COUNT];

    for (int i = 0; i < servoCount; i++)
    {
        int16_t cur = (int16_t)roundf(currentList[i] / DXL_CUR_RESOLUTION);
        dataArray[i] = (uint16_t)cur;
    }
    setServo_SyncWrite(servoList, servoCount, GoalCurrent_ADDR_LB, GoalCurrent_ADDR_HB, GoalCurrent_ByteSize, dataArray);
}

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function to "Syncread" multiple Servo parameter ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

void syncRead_ID(volatile ServoXM4340 *servoList, int servoCount)
{
    getServo_SyncRead(servoList, servoCount, ID_ADDR_LB, ID_ADDR_HB, ID_ByteSize);

    for (int i = 0; i < servoCount; i++)
    {
        servoList[i].ID = (uint8_t)servoList[i].Response.params[0];
    }
}

/**
 * @brief  Get the servo Present Position parameter value at the same time.
 * @param  servoList An array of volatile ServoXM4340 structure
 * @param  servoCount Size of servoArray,which represents the number of servo
 * @param  posList An array of position buffer to save the present position of each servo
 * @retval None
 */
void syncRead_PresentPosition(volatile ServoXM4340 *servoList, int servoCount, float *posList)
{
    getServo_SyncRead(servoList, servoCount, PresentPosition_ADDR_LB, PresentPosition_ADDR_HB, PresentPosition_ByteSize);
    for (int i = 0; i < servoCount; i++)
    {
        int32_t pos = 0;
        pos |= servoList[i].Response.params[0];
        pos |= servoList[i].Response.params[1] << 8;
        pos |= servoList[i].Response.params[2] << 16;
        pos |= servoList[i].Response.params[3] << 24;

        servoList[i].PresentPosition = pos;

        posList[i] = (float)pos * DXL_POS_RESOLUTION;
    }
}

/**
 * @brief  Get the servo PresentCurrent parameter value at the same time.
 * @param  servoList An array of volatile ServoXM4340 structure
 * @param  servoCount Size of servoArray,which represents the number of servo
 * @param  curList An array of current buffer to save the present current of each servo
 * @retval None
 */
void syncRead_PresentCurrent(volatile ServoXM4340 *servoList, int servoCount, float *curList)
{
    getServo_SyncRead(servoList, servoCount, PresentCurrent_ADDR_LB, PresentCurrent_ADDR_HB, PresentCurrent_ByteSize);

    for (int i = 0; i < servoCount; i++)
    {
        int16_t pres_cur = 0;
        pres_cur |= (int)servoList[i].Response.params[0];
        pres_cur |= (int)servoList[i].Response.params[1] << 8;

        servoList[i].PresentCurrent = pres_cur;

        curList[i] = (float)pres_cur * DXL_CUR_RESOLUTION;
    }
}

void syncRead_PresentVelocity(volatile ServoXM4340 *servoList, int servoCount, float *velList)
{
    getServo_SyncRead(servoList, servoCount, PresentVelocity_ADDR_LB, PresentVelocity_ADDR_HB, PresentVelocity_ByteSize);

    for (int i = 0; i < servoCount; i++)
    {
        int32_t pres_vel = 0;
        pres_vel |= (int)servoList[i].Response.params[0];
        pres_vel |= (int)servoList[i].Response.params[1] << 8;
        pres_vel |= (int)servoList[i].Response.params[2] << 16;
        pres_vel |= (int)servoList[i].Response.params[3] << 24;

        servoList[i].PresentVelocity = pres_vel;

        velList[i] = (float)pres_vel * DXL_VEL_RESOLUTION;
    }
}
/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function that should not  be called externally: ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief  Set the servo parameter at the same time.
 * @param  servoArray An array of volatile ServoXM4340 structures
 * @param  servoCount Size of servoArray, which represents the number of servo
 * @param  addrLB Lower byte of servo parameter address
 * @param  addrHB Higherer byte of servo parameter address
 * @param  addrSize Address size of servo parameter
 * @param  dataArray An array of parameter to write into the servo parameter
 * @retval HAL_StatusTypeDef
 * @note  Servo don't return status packet for SyncWrite command with Broadcast ID.
 */
void setServo_SyncWrite(volatile ServoXM4340 *servoArray, int servoCount, uint8_t addrLB, uint8_t addrHB, uint8_t addrSize, uint32_t *dataArray)
{

    // Check all motor have the same UART

    // Set parameter
    uint8_t params_arr[4 + (1 + SERVO_MAX_ADDR_SIZE) * SERVO_MAX_COUNT] = {0};
    params_arr[0] = addrLB;
    params_arr[1] = addrHB;
    params_arr[2] = addrSize;
    params_arr[3] = 0;
    for (int i = 0; i < servoCount; i++)
    {
        int start = 4 + (1 + addrSize) * i;
        params_arr[start] = servoArray[i].ID;
        for (uint8_t j = 1; j <= addrSize; j++)
        {
            params_arr[start + j] = (uint8_t)(dataArray[i] >> (j - 1) * 8) & 0x00ff;
        }
    }

    // TX turn on
    HAL_GPIO_WritePin(servoArray[0].ctrlPort, servoArray[0].ctrlPin, GPIO_PIN_SET);

    sendServoCommand(servoArray[0].huart, ID_broadcast, INSTRUCTION_SYNC_WRITE, 4 + (1 + addrSize) * servoCount, params_arr);

    while (!TxFinished)
    {
        // wait until transmitting finished;
    };
}

/**
 * @brief  Get the servo parameter at the same time.
 * @param  servoArray An array of volatile ServoXM4340 structure
 * @param  servoCount Size of servoArray,which represents the number of servo
 * @param  addrLB Lower byte of servo parameter address
 * @param  addrHB Higherer byte of servo parameter address
 * @param  addrSize Address size of servo parameter
 * @retval HAL status
 * @note  Servo don't return status packet for SyncWrite command with Broadcast ID.
 */
HAL_StatusTypeDef getServo_SyncRead(volatile ServoXM4340 *servoArray, int servoCount, uint8_t addrLB, uint8_t addrHB, uint8_t addrSize)
{

    // Check all motor have the same UART

    // Set parameter
    uint8_t params_arr[4 + SERVO_MAX_COUNT] = {0};
    params_arr[0] = addrLB;
    params_arr[1] = addrHB;
    params_arr[2] = addrSize;
    params_arr[3] = 0;
    for (int i = 0; i < servoCount; i++)
    {
        params_arr[4 + i] = servoArray[i].ID;
    }

    volatile int received[SERVO_MAX_COUNT];
    volatile int state[SERVO_MAX_COUNT];
    for (int i = 0; i < servoCount; i++)
    {
        received[i] = false;
        state[i] = SERVO_OFFLINE;
    }

    do
    {
        // Servo_RxBuffer clear
        for (int i = 0; i < servoCount; i++)
        {
            clear_Servo_RX_buffer(&servoArray[i]);
        }

        // DXL_RxBuffer ready
        HAL_UART_Receive_DMA(servoArray[0].huart, DXL_RxBuffer, servoCount * (SIZE_STATUS_PACKET + addrSize));

        // TX turn on
        HAL_GPIO_WritePin(servoArray[0].ctrlPort, servoArray[0].ctrlPin, GPIO_PIN_SET);

        sendServoCommand(servoArray[0].huart, ID_broadcast, INSTRUCTION_SYNC_READ, 4 + servoCount, params_arr);

        while (!TxFinished)
        {
            // wait until transmitting finished;
        };

        // RX turn on
        HAL_GPIO_WritePin(servoArray[0].ctrlPort, servoArray[0].ctrlPin, GPIO_PIN_RESET);

        RxFinished = false;

        uint32_t tickstart = HAL_GetTick();
        while (!RxFinished)
        {

            if ((HAL_GetTick() - tickstart) > 1000)
            {
                for (int i = 0; i < servoCount; i++)
                {
                    servoArray[i].state = SERVO_OFFLINE;
                    break; // break while
                }
            }
        }

        DXL_AssignRxBufferToServo(servoArray, servoCount, servoCount * (SIZE_STATUS_PACKET + addrSize));

        for (int i = 0; i < servoCount; i++)
        {
            state[i] = servoArray[i].state;
            if (state[i] != SERVO_OFFLINE)
            {
                received[i] = checkServoResponse(&servoArray[i]);
            }
        }

    } while (allTrue(received, servoCount) != true);

    // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // LD3

    if (allTrue(state, servoCount) == true)
        return HAL_OK;
    else
        return HAL_ERROR;
}

void dualTransferServo(volatile ServoXM4340 *servo, int instructionType, int packet_size, uint8_t *params_arr, int param_size)
{
    do
    {
        // RxBuffer clear
        clear_Servo_RX_buffer(servo);

        // data received and processed in Uart_Callback function
        // DXL_RxBuffer ready
        HAL_UART_Receive_DMA(servo->huart, DXL_RxBuffer, packet_size);

        // TX turn on
        HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_SET);

        sendServoCommand(servo->huart, servo->ID, instructionType, param_size, params_arr);

        uint32_t tickstart = HAL_GetTick();
        while (!TxFinished)
        {
            // wait until transmitting finished;
            if ((HAL_GetTick() - tickstart) > 1500)
            {
                servo->state = SERVO_OFFLINE;
                break; // break while
            }
        };

        // RX turn on
        HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_RESET);

        // wait for receiving completed
        getServoResponse(servo, packet_size);
        if (servo->state == SERVO_OFFLINE)
            break;

    } while (!checkServoResponse(servo)); // redo if checking is not OK
}

uint16_t sendServoCommand(UART_HandleTypeDef *huart, uint8_t servoId, uint8_t commandByte, uint8_t numParams, uint8_t *params)
{

    clear_TX_buffer();
    TxFinished = false;

    // assign byte value to packet
    /*------------------------------------------------------------------------*/
    DXL_TxBuffer[0] = 0xff;                   // header1
    DXL_TxBuffer[1] = 0xff;                   // header2
    DXL_TxBuffer[2] = 0xfd;                   // header3
    DXL_TxBuffer[3] = 0x00;                   // reserved
    DXL_TxBuffer[4] = (uint8_t)servoId;       // ID
    DXL_TxBuffer[5] = (uint8_t)numParams + 3; // packet length(low byte) = byte sizeof  instruction(1) + parameter +CRC(2)
    DXL_TxBuffer[6] = 0x00;                   // packet length(high byte), because numParams set to be uint8_t
    DXL_TxBuffer[7] = (uint8_t)commandByte;   // instruction, ex:Ping, Resd, Write...
    for (uint8_t i = 0; i < numParams; i++)
    {
        DXL_TxBuffer[8 + i] = (uint8_t)params[i];
    }

    // CRC calculation
    /*------------------------------------------------------------------------*/
    uint16_t crc_val = updateCRC(0, DXL_TxBuffer, DXL_TxBuffer[5] + 5);
    DXL_TxBuffer[8 + numParams] = (uint8_t)crc_val & 0x00ff;              // CRC lower byte
    DXL_TxBuffer[8 + numParams + 1] = (uint8_t)(crc_val >> 8) & (0x00ff); // CRC higher byte

    // Data transmitting
    /*------------------------------------------------------------------------*/
    HAL_UART_Transmit_DMA(huart, DXL_TxBuffer, 8 + numParams + 2);

    return crc_val;
}

void getServoResponse(volatile ServoXM4340 *servo, uint16_t RxLen)
{
    RxFinished = false;

    uint32_t tickstart = HAL_GetTick();

    // loop until Receive confirmed
    while (!RxFinished)
    {

        if ((HAL_GetTick() - tickstart) > 1000)
        {
            servo->state = SERVO_OFFLINE;
            break; // break while
        }
    }

    DXL_AssignRxBufferToServo(servo, 1, RxLen);
}

void clear_Servo_RX_buffer(volatile ServoXM4340 *servo)
{
    for (int i = 0; i < SERVO_MAX_RX_BUFFER_SIZE; i++)
    {
        servo->Response.RxBuffer[i] = 0;
    }
}
void clear_DXL_RX_buffer(int dataLen)
{
    for (int i = 0; i < dataLen; i++)
    {
        DXL_RxBuffer[i] = 0;
    }
}

void clear_TX_buffer(void)
{
    for (int i = 0; i < sizeof(DXL_TxBuffer); i++)
    {
        DXL_TxBuffer[i] = 0;
    }
}

bool allTrue(int arr[], int len)
{
    for (int i = 0; i < len; i++)
    {
        if (arr[i] == 0 || arr[i] == SERVO_OFFLINE)
            return false;
    }
    return true;
}

bool checkServoResponse(volatile ServoXM4340 *servo)
{

    if (servo->Response.RxBuffer[0] == 0xff && servo->Response.RxBuffer[1] == 0xff && servo->Response.RxBuffer[2] == 0xfd)
    {
        if (servo->Response.RxBuffer[INDEX_SATUS_PACKET_ID] == servo->ID)
        {
            uint8_t RxdPacketLen = servo->Response.RxBuffer[INDEX_SATUS_PACKET_LEN_L] + 5;
            uint16_t crc_val = updateCRC(0, servo->Response.RxBuffer, RxdPacketLen);
            servo->Response.crc[0] = (uint8_t)crc_val & 0x00ff;          // CRC low byte
            servo->Response.crc[1] = (uint8_t)(crc_val >> 8) & (0x00ff); // CRC high byte->
            if (servo->Response.RxBuffer[RxdPacketLen] == servo->Response.crc[0] && servo->Response.RxBuffer[RxdPacketLen + 1] == servo->Response.crc[1])
            {
                servo->Response.id = servo->Response.RxBuffer[INDEX_SATUS_PACKET_ID];
                servo->Response.length = servo->Response.RxBuffer[INDEX_SATUS_PACKET_LEN_L];
                servo->Response.error = servo->Response.RxBuffer[INDEX_SATUS_PACKET_ERR];
                for (int i = 0; i < servo->Response.length - 4; i++)
                {
                    servo->Response.params[i] = servo->Response.RxBuffer[9 + i];
                }
                servo->state = SERVO_ONLINE;
                return true;
            }
            else
            {
                servo->state = SERVO_OFFLINE;
                return false;
            }
        }
        else
        {
            servo->state = SERVO_OFFLINE;
            return false;
        }
    }
    else
    {
        servo->state = SERVO_OFFLINE;
        return false;
    }
}

unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i, j;
    static const uint16_t crc_table[256] = {0x0000,
                                            0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                                            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
                                            0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
                                            0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
                                            0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
                                            0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
                                            0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
                                            0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
                                            0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                                            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
                                            0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
                                            0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
                                            0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
                                            0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
                                            0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
                                            0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
                                            0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                                            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
                                            0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
                                            0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
                                            0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
                                            0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
                                            0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
                                            0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
                                            0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                                            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
                                            0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
                                            0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
                                            0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
                                            0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
                                            0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
                                            0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
                                            0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                                            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
                                            0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
                                            0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
                                            0x820D, 0x8207, 0x0202};

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

__weak void disable_all_IT()
{
}

__weak void enable_all_IT()
{
}