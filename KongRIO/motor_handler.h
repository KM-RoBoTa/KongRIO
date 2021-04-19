#ifndef MOTOR_HANDLER
#define MOTOR_HANDLER

#include <DynamixelWorkbench.h>

typedef int32_t joint_value;

class MotorHandler
{
  DynamixelWorkbench *comm_;

public:
  uint8_t id;
  String model;  

  void SetInterface(DynamixelWorkbench* comm);
  
  //void JointMode(uint16_t aCWLimit=312, uint16_t aCCWLimit=712);  //NOT SURE BUT THIS FUNCTION HAS WRONG ARGUMENTS....Original:void JointMode(uint16_t aCWLimit=0, uint16_t aCCWLimit=0x3FF); Current: +/- 60 degree.
  
  void EnableTorque(bool aTorque=true);

  bool GoalPosition(joint_value aPosition);
  bool TorqueLimit(int32_t aTorque);
  bool PosPGain(int32_t aPosPGain);
  
  joint_value CurrentPosition();
  int32_t CurrentTemperature();                               //added to get the status of the T and V in the motors
  int32_t CurrentVoltage();

};

#endif
