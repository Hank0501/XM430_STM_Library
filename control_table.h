/**
 * @file    control_table.h
 * @author  Z.H. Wu
 * @brief   Control table of Dynamixel XM430 servo, see
 *          https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
 *
 * @note    This file does not contain all parameter macro for XM430 servo,
 *          only commonly used are recorded.
 */

#ifndef CONTROL_TABLE_H
#define CONTROL_TABLE_H

#define DXL_POS_RESOLUTION 0.0878906 // deg
#define DXL_CUR_RESOLUTION 2.69      // mA

// XM4340 control table
#define GoalPosition_ADDR_LB 0x74
#define GoalPosition_ADDR_HB 0x00
#define GoalPosition_ByteSize 4

#define OperatingMode_ADDR_LB 0x0b
#define OperatingMode_ADDR_HB 0x00
#define OperatingMode_ByteSize 1
#define Current_CtrlMode 0
#define Velocity_CtrlMode 1
#define POS_CtrlMode 3
#define Extended_Pos_CtrlMode 4
#define CurrentBased_POS_CtrlMode 5
#define PWM_CtrlMode 16

#define TorqueEnable_ADDR_LB 0x40
#define TorqueEnable_ADDR_HB 0x00
#define TorqueEnable_ByteSize 1
#define TORQUE_ENABLE 1  // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque

#define PresentCurrent_ADDR_LB 0x7E
#define PresentCurrent_ADDR_HB 0x00
#define PresentCurrent_ByteSize 2

#define GoalCurrent_ADDR_LB 0x66
#define GoalCurrent_ADDR_HB 0x00
#define GoalCurrent_ByteSize 2

#define CurrentLimit_ADDR_LB 0x26
#define CurrentLimit_ADDR_HB 0x00
#define CurrentLimit_ByteSize 2

#define BaudRate_ADDR_LB 0x08
#define BaudRate_ADDR_HB 0x00
#define BaudRate_ByteSize 1
#define BaudRate_57600 1
#define BaudRate_115200 2
#define BaudRate_1M 3
#define BaudRate_2M 4
#define BaudRate_3M 5
#define BaudRate_4M 6
#define BaudRate_4p5M 7

#define ID_ADDR_LB 0x07
#define ID_ADDR_HB 0x00
#define ID_ByteSize 1
#define ID_Default 1
#define ID_broadcast 254

#define PresentPosition_ADDR_LB 0x84
#define PresentPosition_ADDR_HB 0x00
#define PresentPosition_ByteSize 4

#define ProfileAcceleration_ADDR_LB 0x6C
#define ProfileAcceleration_ADDR_HB 0x00
#define ProfileAcceleration_ByteSize 4
#define ProfileAccelerationLimit_V_Based 32767
#define ProfileAccelerationLimit_T_Based 32737

#define ProfileVelocity_ADDR_LB 0x70
#define ProfileVelocity_ADDR_HB 0x00
#define ProfileVelocity_ByteSize 4
#define ProfileVelocityLimit_V_Based 32767
#define ProfileVelocityLimit_T_Based 32737

#define DriveMode_ADDR_LB 0x0A
#define DriveMode_ADDR_HB 0x00
#define DriveMode_ByteSize 1

#endif