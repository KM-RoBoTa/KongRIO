#ifndef MOTORS_MANAGER
#define MOTORS_MANAGER

#include <DynamixelWorkbench.h>
#include "motor_handler.h"

#define DEVICE_NAME "3" //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
#define BAUDRATE  1000000
//#define DXL_ID 16 //used only with a single motor
#define NUM_MOTORS 8

struct MotorArray {
  void reset() {
    for (int i = 0; i<NUM_MOTORS; i++) {
      values[i] = 0;
    }
  }
  joint_value values[NUM_MOTORS];                                   //Kamilo: Why are we setting this to 0?
  static const int size = NUM_MOTORS;
};

class MotorManager {
private:
  DynamixelWorkbench dxl_wb;
  
  MotorHandler motors[NUM_MOTORS];
  uint8_t scanned_id[NUM_MOTORS] = {1,2,3,4,5,6,7,8}; //all motors ID's in the robot
  //uint8_t scanned_id[NUM_MOTORS] = {21,22,23,24,1,2,11,12,13,14,3,4,5,6,7,8}; //all motors ID's in the robot
  //uint8_t scanned_id[NUM_MOTORS] = {21, 22, 23, 24}; //all motors ID's in the robot
  bool Init() {
    bool result = dxl_wb.begin(DEVICE_NAME, BAUDRATE);
    
    if (result == false)
    {
      Serial.println("Failed to init");
    }
    else
    {
      Serial.print("Succeeded to init : ");
      Serial.println(BAUDRATE);  
    }
    return result;
  }
  
  bool FindDevices() {

    //    dxl_wb.ping(DXL_ID);                                    //used only with a single motor
    //    dxl_wb.jointMode(DXL_ID);

    const char *log = NULL;
    bool result = false;
  

    uint8_t dxl_cnt = NUM_MOTORS;
    uint8_t range = 100;
  
   // Serial.println("Wait for scan...");                         //used to scan motors in the chain. Disabled as it is assumed that all motors are properly working.
   // result = dxl_wb.scan(scanned_id, &dxl_cnt, range, &log);
   // if (result == false)
   // {
   //   Serial.println(log);
   //   Serial.println("Failed to scan")
   // }
   // else
    {
      Serial.print("Find ");
      Serial.print(dxl_cnt);
      Serial.println(" Dynamixels");
  
      for (int cnt = 0; cnt < dxl_cnt; cnt++)
      {
        Serial.print("id : ");
        Serial.print(scanned_id[cnt]);
        Serial.print(" model name : ");
        Serial.println(dxl_wb.getModelName(scanned_id[cnt]));
        motors[cnt].id = scanned_id[cnt]; 
        motors[cnt].model = dxl_wb.getModelName(scanned_id[cnt]);
        motors[cnt].SetInterface(&dxl_wb);
        dxl_wb.ping(scanned_id[cnt]);
        dxl_wb.jointMode(scanned_id[cnt]);
      }
    }
  }


public:

  void Setup()
  {
    Init();
    FindDevices();               
    dxl_wb.addSyncReadHandler(scanned_id[0], "Present_Position");         //handlers to read from all motors at once
    dxl_wb.addSyncWriteHandler(scanned_id[0], "Goal_Position");
   
  }

  MotorArray GetCurrentPositions() {
    MotorArray result;
    //const uint8_t handler_index = 0;                                      //fuctions to read from all motors at once
    //dxl_wb.syncRead(handler_index);
    //dxl_wb.getSyncReadData(handler_index,&result.values[0]);
    for (int i = 0; i < NUM_MOTORS; i++){                               //fucntion to read from single motors
      result.values[i] = motors[i].CurrentPosition();
    }
    return result;
  }
  
  void SetPositions(MotorArray positions) {
    const uint8_t handler_index = 0;                                      //fuctions to write to all motors at once
    dxl_wb.syncWrite(handler_index, &positions.values[0]);
    //for (int i = 0; i < NUM_MOTORS; i++){                               //fucntion to write to single motors
      //motors[i].GoalPosition(positions.values[i]);
    //}
  }

  void SetTorqueLimit(int32_t value) {
    for (int i = 0; i < NUM_MOTORS; i++){                               //fucntion to write to single motors Not used with protocol 2
      motors[i].TorqueLimit(value);
    }
  }

  void SetPosPGain(int32_t value) {
    for (int i = 0; i < NUM_MOTORS; i++){                               //fucntion to write to single motors Not used with protocol 2
      motors[i].PosPGain(value);
    }
  }


  int32_t GetHighestTemperature() {
    int32_t result = 0;
    for (int i = 0; i < NUM_MOTORS; i++){                                //fucntion to read from motors the Max temperature
      int32_t temp = motors[i].CurrentTemperature();
      if (result < temp) {
          result = temp;
      }
    }
    return result;
  }

  int32_t GetLowestVoltage() {
    int32_t result = 140;
    int32_t sum = 0;
    for (int i = 0; i < NUM_MOTORS; i++){ //fucntion to read from motors the average voltage
      int32_t c_voltage = motors[i].CurrentVoltage(); //side effect functions should be cached
      if (result > c_voltage) { //Do you need to filter here? otherwise sum = sum + c_voltage; result = sum / NUM_MOTORS; is way faster
        result = (result*(i) + c_voltage)/(i+1);
      }
    }
    return result;
  }

};

#endif
