# my_dynamixel Library

## Introduction

- This library is used for communication between STM32 and Dynamixel XM430 servo.

## Description

- STM32 USART functoin is used.
- This library is a blocking version, STM32 will not do anything untill a complete transmitting and receiving have been finished.
- For all the motor-related function, the status packet from motor should be received in less than 1 sec, or the servo state turns to be SERVO_OFFLINE (value = 0xFF)
- A complete process of transmitting and receiving is
  1. The packet is transmitting through the USART.
  2. Wait for the status packet.
  3. If waiting for longer than 1 sec, the servo state turns to be 0xFF.
  4. If the receiving is success, then check the CRC of the packet.
  5. If the CRC packet isn't correct, jump to step 1.(resent the data), or the whole process is done here.

## Undone List

- Syncread, Syncwrite..., may write a new dualTransferServo for sync-case, original one for one servo situation.
- the non-blocking version
- SyncReadID
- DMA version is still needed!!!!!

# ! Attention !

> Below are some important detail to use the motor, which are not recorded in the XM430-W350 Manual.

1. The GoalXXXX parameter (_ex: GoalCurrent, GoalVelocity_) will be reset to the corresponding limit value after setting the XXXXLimit parameter (_ex: CurrentLimit, VelocityLimit_)
2. The GoalXXXX parameter can't be written when the motor is in corresponding mode but the TorqueEnable parameter is off
   > ex : If the motor is now in Velocity control mode.
   >
   > > This will not work :
   > >
   > > 1. the TorqueEnable parameter is originally off
   > > 2. set the GoalVelocity parameter
   > > 3. set the TorqueEnable parameter on
   > > 4. the GoalVelocity parameter is not written
   >
   > > This will work :
   > >
   > > 1. the TorqueEnable parameter is originally off
   > > 2. set the TorqueEnable parameter on
   > > 3. set the GoalVelocity parameter
   > > 4. the GoalVelocity parameter has been written
