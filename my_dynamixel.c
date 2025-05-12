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

volatile uint8_t DXL_RxBuffer[SERVO_MAX_RX_BUFFER_SIZE];

/*=================================================================================================*/
/*=================================================================================================*/
/*======================== Function that user can reach Servo structure ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief  Set the RxFinished flag to check whether data received from servo.
 * @param  servo ServoXM430 structure
 * @param  val booling value
 *      @arg  true: data received completed
 *      @arg  false: data received incompleted
 * @retval  None
 */
void DXL_SetServoResponse_RxFinished(ServoXM4340 *servo, bool val)
{
    servo->Response.RxFinished = val;
}

void DXL_AssignRxBufferToServo(ServoXM4340 *servo)
{

    for (int i = 0; i < servo->Response.RxBuffer[INDEX_SATUS_PACKET_LEN_L] + 7; i++)
    {
        servo->Response.RxBuffer[i] = DXL_RxBuffer[i];
    }
}

uint8_t DXL_GetRxBufferID(void)
{
    return DXL_RxBuffer[INDEX_SATUS_PACKET_ID];
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
void servo_FactoryReset(ServoXM4340 *servo, uint8_t packetID, uint8_t resetvalue)
{

    // parameters calculated and send the instruction
    uint8_t params_arr[1] = {0};
    params_arr[0] = resetvalue;

    // TX turn on
    HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_SET);

    sendServoCommand(servo->huart, packetID, INSTRUCTION_FactoryReset, 1, params_arr);

    // RX turn on
    HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_RESET);

    HAL_UART_DeInit(servo->huart);
    (*(servo->huart)).Init.BaudRate = 57600;
    if (HAL_UART_Init(servo->huart) != HAL_OK)
    {
    }

    // The DYNAMIXEL Broadcast ID (254 (0xFE)) will only return Status Packets for Ping, Sync Read and Bulk Read commands
    if (packetID != ID_broadcast)
    {
        getServoResponse(servo, SIZE_STATUS_PACKET);
        checkServoResponse(servo);
    }
    else
    {
        HAL_UART_Receive_IT(servo->huart, servo->Response.RxBuffer, SIZE_STATUS_PACKET);
    }
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
void setServo_BaudRate(ServoXM4340 *servo, uint8_t baud)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[3] = {0};
    params_arr[0] = BaudRate_ADDR_LB;
    params_arr[1] = BaudRate_ADDR_HB;
    params_arr[2] = baud;

    // TX turn on
    HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_SET);

    sendServoCommand(servo->huart, servo->ID, INSTRUCTION_WRITE, 3, params_arr);

    // RX turn on
    HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_RESET);

    getServoResponse(servo, SIZE_STATUS_PACKET);
    checkServoResponse(servo);
}

/**
 * @brief  Set the servo ID parameter.
 * @param  servo Pointer to ServoXM430 structure
 * @param  id Baudrate value
 * @retval  None
 * @note  The servo return status packet in original ID value
 */
void setServo_ID(ServoXM4340 *servo, uint8_t id)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[3] = {0};
    params_arr[0] = ID_ADDR_LB;
    params_arr[1] = ID_ADDR_HB;
    params_arr[2] = id;

    // TX turn on
    HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_SET);

    sendServoCommand(servo->huart, servo->ID, INSTRUCTION_WRITE, 3, params_arr);

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
void setServo_OperatingMode(ServoXM4340 *servo, uint8_t operatingMode)
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
void setServo_TorqueENA(ServoXM4340 *servo, uint8_t torque)
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
 * @param  current Goal current value
 * @retval  None
 * @note  Goal current value should not exceed Current Limit value
 */
void setServo_GoalCurrent(ServoXM4340 *servo, uint16_t current)
{

    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = GoalCurrent_ADDR_LB;
    params_arr[1] = GoalCurrent_ADDR_HB;
    params_arr[2] = (uint8_t)current & 0x00ff;
    params_arr[3] = (uint8_t)(current >> 8) & (0x00ff);

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

/**
 * @brief  Set the servo Goal position parameter.
 * @param  servo ServoXM430 structure
 * @param  angle Goal position value, in degree unit
 * @retval  None
 * @note  Goal position value has limitation in different operrating mode, see manual.
 */
void setServo_GoalPosition(ServoXM4340 *servo, const float angle)
{
    // parameters calculated and send the instruction
    // const float angleResolution = (float)360 / 4096;
    int32_t temp = angle / 0.08789; // 0.08789 = 360/4096
    uint32_t angle_cmd = temp;
    uint8_t params_arr[6] = {0};
    params_arr[0] = GoalPosition_ADDR_LB;
    params_arr[1] = GoalPosition_ADDR_HB;
    params_arr[2] = (uint8_t)angle_cmd & 0x000000ff;
    params_arr[3] = (uint8_t)(angle_cmd >> 8) & (0x000000ff);
    params_arr[4] = (uint8_t)(angle_cmd >> 16) & (0x000000ff);
    params_arr[5] = (uint8_t)(angle_cmd >> 24) & (0x000000ff);

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

/**
 * @brief  Set the servo Current Limit parameter.
 * @param  servo ServoXM430 structure
 * @param  current Current limit value
 * @retval  None
 * @note  Range of current limit value is 0~1193 for XM430
 */
void setServo_CurrentLimit(ServoXM4340 *servo, uint16_t current)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = CurrentLimit_ADDR_LB;
    params_arr[1] = CurrentLimit_ADDR_HB;
    params_arr[2] = (uint8_t)current & 0x00ff;
    params_arr[3] = (uint8_t)(current >> 8) & (0x00ff);

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

void setServo_ProfileAcceleration(ServoXM4340 *servo, uint16_t maxAcc)
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

void setServo_ProfileVelocity(ServoXM4340 *servo, uint16_t maxVel)
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

void setServo_DriveMode(ServoXM4340 *servo, uint8_t conf)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[3] = {0};
    params_arr[0] = DriveMode_ADDR_LB;
    params_arr[1] = DriveMode_ADDR_HB;
    params_arr[2] = conf;

    dualTransferServo(servo, INSTRUCTION_WRITE, SIZE_STATUS_PACKET, params_arr, sizeof(params_arr));
}

/*=================================================================================================*/
/*=================================================================================================*/
/*===========================  Function to Read single Servo parameter  ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief  Get the servo Baudrate value.
 * @param  servo ServoXM430 structure
 * @retval  Baudrate value
 */
int getServo_BaudRate(ServoXM4340 *servo)
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
        return 57600;
        break;
    case BaudRate_115200:
        return 115200;
        break;
    case BaudRate_1M:
        return 1000000;
        break;
    case BaudRate_2M:
        return 2000000;
        break;
    case BaudRate_3M:
        return 3000000;
        break;
    case BaudRate_4M:
        return 4000000;
        break;
    case BaudRate_4p5M:
        return 4500000;
        break;
    default:
        break;
    }
    return 0;
}

uint8_t getServo_DriveMode(ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = DriveMode_ADDR_LB;
    params_arr[1] = DriveMode_ADDR_HB;
    params_arr[2] = DriveMode_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + DriveMode_ByteSize, params_arr, sizeof(params_arr));

    return servo->Response.params[0];
}

/**
 * @brief  Get the servo TorqueEnable parameter value.
 * @param  servo ServoXM430 structure
 * @retval  TorqueEnable parameter value, 1 for torque on and 0 for torque off
 */
uint8_t getServo_TorqueENA(ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = TorqueEnable_ADDR_LB;
    params_arr[1] = TorqueEnable_ADDR_HB;
    params_arr[2] = TorqueEnable_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + TorqueEnable_ByteSize, params_arr, sizeof(params_arr));

    return servo->Response.params[0];
}

/**
 * @brief  Get the servo PresentCurrent parameter value.
 * @param  servo ServoXM430 structure
 * @retval  Present Current parameter value, the unit of the value is approximately 2.69 mA, see manual.
 */
int16_t getServo_PresentCurrent(ServoXM4340 *servo)
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
    pre_cur |= (int)servo->Response.params[0];
    pre_cur |= (int)servo->Response.params[1] << 8;
    return pre_cur;
}

/**
 * @brief  Get the servo Goal Current parameter value.
 * @param  servo ServoXM430 structure
 * @retval  Goal Current parameter value, the unit of the value is approximately 2.69 mA, see manual.
 * @
 */
int16_t getServo_GoalCurrent(ServoXM4340 *servo)
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
    goal_cur |= (int)servo->Response.params[0];
    goal_cur |= (int)servo->Response.params[1] << 8;
    return goal_cur;
}

/**
 * @brief  Get the servo Current Limit parameter value.
 * @param  servo ServoXM430 structure
 * @retval  Current Limit parameter value, the unit of the value is approximately 2.69 mA, see manual.
 */
uint16_t getServo_CurrentLimit(ServoXM4340 *servo)
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
    cur |= (int)servo->Response.params[0];
    cur |= (int)servo->Response.params[1] << 8;
    return cur;
}

/**
 * @brief  Get the servo Present Position value.
 * @param  servo ServoXM430 structure
 * @retval Present Position value deg unit
 */
float getServo_PresentPosition(ServoXM4340 *servo)
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
    pos |= servo->Response.params[0];
    pos |= servo->Response.params[1] << 8;
    pos |= servo->Response.params[2] << 16;
    pos |= servo->Response.params[3] << 24;

    servo->Position = pos;

    return (float)pos * 360.0 / 4096.0;
}

/**
 * @brief  Get the servo Operating Mode parameter value.
 * @param  servo ServoXM430 structure
 * @retval  OperatingMode parameter value
 * @retval  0   Current control mode
 * @retval  1   Velocity control mode
 * @retval  3   Position control mode
 * @retval  4   Extended position control mode
 * @retval  16  PWM control mode
 */
uint8_t getServo_OperatingMode(ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = OperatingMode_ADDR_LB;
    params_arr[1] = OperatingMode_ADDR_HB;
    params_arr[2] = OperatingMode_ByteSize;
    params_arr[3] = 0x00;

    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + OperatingMode_ByteSize, params_arr, sizeof(params_arr));

    return servo->Response.params[0];
}

uint16_t getServo_ProfileAcceleration(ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = ProfileAcceleration_ADDR_LB;
    params_arr[1] = ProfileAcceleration_ADDR_HB;
    params_arr[2] = ProfileAcceleration_ByteSize;
    params_arr[3] = 0x00;
    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + ProfileAcceleration_ByteSize, params_arr, sizeof(params_arr));

    // record the current  of the servo
    uint16_t prof = 0;
    prof |= servo->Response.params[0];
    prof |= servo->Response.params[1] << 8;
    return prof;
}

uint16_t getServo_ProfileVelocity(ServoXM4340 *servo)
{
    // parameters calculated and send the instruction
    uint8_t params_arr[4] = {0};
    params_arr[0] = ProfileVelocity_ADDR_LB;
    params_arr[1] = ProfileVelocity_ADDR_HB;
    params_arr[2] = ProfileVelocity_ByteSize;
    params_arr[3] = 0x00;
    dualTransferServo(servo, INSTRUCTION_READ, SIZE_STATUS_PACKET + ProfileVelocity_ByteSize, params_arr, sizeof(params_arr));

    // record the current  of the servo
    uint16_t prof = 0;
    prof |= servo->Response.params[0];
    prof |= servo->Response.params[1] << 8;
    return prof;
}

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function to "Syncwrite" multiple Servo parameter ==========================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief  Set the servo Goal position parameter at the same time.
 * @param  servoList An array of ServoXM4340 structures
 * @param  servoCount Size of servoArray, which represents the number of servo
 * @param  angleList An array of angle command to write into each servo
 * @retval  None
 * @note  Goal position value has limitation in different operrating mode, see manual.
 * @note  Servos don't return status packet for SyncWrite instruction.
 */
void syncWrite_GoalPosition(ServoXM4340 *servoList, int servoCount, const float *angleList)
{
    uint32_t dataArray[SERVO_MAX_COUNT];
    for (int i = 0; i < servoCount; i++)
    {
        int32_t temp = angleList[i] / 0.08789; // 0.08789 = 360/4096
        uint32_t angle_cmd = temp;             // in 2's complement
        dataArray[i] = angle_cmd;
    }
    setServo_SyncWrite(servoList, servoCount, GoalPosition_ADDR_LB, GoalPosition_ADDR_HB, GoalPosition_ByteSize, dataArray);
}

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function to "Syncread" multiple Servo parameter ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief  Get the servo Present Position parameter value at the same time.
 * @param  servoList An array of ServoXM4340 structure
 * @param  servoCount Size of servoArray,which represents the number of servo
 * @param  posList An array of position buffer to save the present position of each servo
 * @retval None
 */
void syncRead_PresentPosition(ServoXM4340 *servoList, int servoCount, float *posList)
{
    getServo_SyncRead(servoList, servoCount, GoalPosition_ADDR_LB, GoalPosition_ADDR_HB, GoalPosition_ByteSize);
    for (int i = 0; i < servoCount; i++)
    {
        int32_t pos = 0;
        pos |= servoList[i].Response.params[0];
        pos |= servoList[i].Response.params[1] << 8;
        pos |= servoList[i].Response.params[2] << 16;
        pos |= servoList[i].Response.params[3] << 24;

        servoList[i].Position = pos;

        posList[i] = (float)pos * 360.0 / 4096.0;
    }
}

/*=================================================================================================*/
/*=================================================================================================*/
/*===================== Function that should not  be called externally: ===========================*/
/*=================================================================================================*/
/*=================================================================================================*/

/**
 * @brief  Set the servo parameter at the same time.
 * @param  servoArray An array of ServoXM4340 structures
 * @param  servoCount Size of servoArray, which represents the number of servo
 * @param  addrLB Lower byte of servo parameter address
 * @param  addrHB Higherer byte of servo parameter address
 * @param  addrSize Address size of servo parameter
 * @param  dataArray An array of parameter to write into the servo parameter
 * @retval HAL_StatusTypeDef
 * @note  Servo don't return status packet for SyncWrite command with Broadcast ID.
 */
void setServo_SyncWrite(ServoXM4340 *servoArray, int servoCount, uint8_t addrLB, uint8_t addrHB, uint8_t addrSize, uint32_t *dataArray)
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
}

/**
 * @brief  Get the servo parameter at the same time.
 * @param  servoArray An array of ServoXM4340 structure
 * @param  servoCount Size of servoArray,which represents the number of servo
 * @param  addrLB Lower byte of servo parameter address
 * @param  addrHB Higherer byte of servo parameter address
 * @param  addrSize Address size of servo parameter
 * @retval HAL status
 * @note  Servo don't return status packet for SyncWrite command with Broadcast ID.
 */
HAL_StatusTypeDef getServo_SyncRead(ServoXM4340 *servoArray, int servoCount, uint8_t addrLB, uint8_t addrHB, uint8_t addrSize)
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
        received[i] = true;
        state[i] = SERVO_ONLINE;
    }

    do
    {
        // TX turn on
        HAL_GPIO_WritePin(servoArray[0].ctrlPort, servoArray[0].ctrlPin, GPIO_PIN_SET);

        sendServoCommand(servoArray[0].huart, ID_broadcast, INSTRUCTION_SYNC_READ, 4 + servoCount, params_arr);

        // RX turn on
        HAL_GPIO_WritePin(servoArray[0].ctrlPort, servoArray[0].ctrlPin, GPIO_PIN_RESET);

        for (int i = 0; i < servoCount; i++)
        {
            // wait for receiving completed
            getServoResponse(&servoArray[i], SIZE_STATUS_PACKET + addrSize);
        }

        for (int i = 0; i < servoCount; i++)
        {
            state[i] = servoArray[i].state;
            if (state[i] != SERVO_OFFLINE)
            {
                received[i] = checkServoResponse(&servoArray[i]);
            }
        }
    } while (allTrue(received, servoCount) != true);

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

    if (allTrue(state, servoCount) == true)
        return HAL_OK;
    else
        return HAL_ERROR;
}

void dualTransferServo(ServoXM4340 *servo, int instructionType, int packet_size, uint8_t *params_arr, int param_size)
{
    do
    {
        // TX turn on
        HAL_GPIO_WritePin(servo->ctrlPort, servo->ctrlPin, GPIO_PIN_SET);

        sendServoCommand(servo->huart, servo->ID, instructionType, param_size, params_arr);

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

    uint8_t TxdPacket[SERVO_MAX_TX_BUFFER_SIZE];
    uint8_t crc[2] = {0};

    // assign byte value to packet
    /*------------------------------------------------------------------------*/
    TxdPacket[0] = 0xff;                   // header1
    TxdPacket[1] = 0xff;                   // header2
    TxdPacket[2] = 0xfd;                   // header3
    TxdPacket[3] = 0x00;                   // reserved
    TxdPacket[4] = (uint8_t)servoId;       // ID
    TxdPacket[5] = (uint8_t)numParams + 3; // packet length(low byte) = byte sizeof  instruction(1) + parameter +CRC(2)
    TxdPacket[6] = 0x00;                   // packet length(high byte), because numParams set to be uint8_t
    TxdPacket[7] = (uint8_t)commandByte;   // instruction, ex:Ping, Resd, Write...
    for (uint8_t i = 0; i < numParams; i++)
    {
        TxdPacket[8 + i] = (uint8_t)params[i];
    }

    // CRC calculation
    /*------------------------------------------------------------------------*/
    uint16_t crc_val = updateCRC(0, TxdPacket, TxdPacket[5] + 5);
    crc[0] = (uint8_t)crc_val & 0x00ff;          // CRC low byte
    crc[1] = (uint8_t)(crc_val >> 8) & (0x00ff); // CRC high byte

    // Data transmitting
    /*------------------------------------------------------------------------*/
    HAL_UART_Transmit(huart, TxdPacket, 8 + numParams, 0xFFFF);
    disable_all_IT();
    HAL_UART_Transmit(huart, crc, 2, 0xFFFF);
    enable_all_IT();

    return crc_val;
}

void getServoResponse(ServoXM4340 *servo, uint16_t RxLen)
{
    servo->Response.RxFinished = false;
    clear_RX_buffer(servo);
    // data received and processed in Uart_Callback function
    HAL_UART_Receive_IT(servo->huart, DXL_RxBuffer, RxLen);

    uint32_t tickstart = HAL_GetTick();

    // loop until Receive confirmed
    while (!servo->Response.RxFinished)
    {
        if ((HAL_GetTick() - tickstart) > 1000)
        {
            servo->state = SERVO_OFFLINE;
            break; // break while
        }
    }
    if (servo->Response.RxFinished)
        servo->state = SERVO_ONLINE;
}

void clear_RX_buffer(ServoXM4340 *servo)
{
    for (int i = 0; i < SERVO_MAX_RX_BUFFER_SIZE; i++)
    {
        servo->Response.RxBuffer[i] = 0;
        DXL_RxBuffer[i] = 0;
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

bool checkServoResponse(ServoXM4340 *servo)
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
                return true;
            }
            else
                return false;
        }
        else
            return false;
    }
    else
        return false;
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