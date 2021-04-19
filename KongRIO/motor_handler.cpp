#include "motor_handler.h"

void MotorHandler::SetInterface(DynamixelWorkbench* comm) {
  comm_ = comm;
}

//void MotorHandler::JointMode(uint16_t aCWLimit, uint16_t aCCWLimit) {
  
//}

void MotorHandler::EnableTorque(bool aTorque) {
  
}


bool MotorHandler::GoalPosition(joint_value aPosition) {
  return comm_->goalPosition(id, aPosition);
}

bool MotorHandler::TorqueLimit(int32_t value) {
  return comm_->itemWrite(id, "Torque_Limit", value);
}

bool MotorHandler::PosPGain(int32_t value) {
  return comm_->itemWrite(id, "Position_P_Gain", value);
}

joint_value MotorHandler::CurrentPosition() {
  joint_value get_data;
  bool result = comm_->itemRead(id, "Present_Position", &get_data);
  return get_data;
}

int32_t MotorHandler::CurrentTemperature() {                            // added to read V and T from the motors
  int32_t get_data;
  bool result = comm_->itemRead(id, "Present_Temperature", &get_data);
  return get_data;
}

int32_t MotorHandler::CurrentVoltage() {
  int32_t get_data;
  bool result = comm_->itemRead(id, "Present_Voltage", &get_data);
  return get_data;
}
