#include <DynamixelSDK.h>

#include <OpenCM904.h>

#include "input_manager.h"
#include "motors_manager.h"
#include "control_laws.h"

InputManager inputs;
OutputManager outputs;
MotorManager motors;

BaseControlLaw* control_laws[CONTROL_LAWS_SIZE];

ControlLaw0 lawHome;
ControlLaw1 law1;
ControlLaw2 law2;
ControlLaw3 law3;
ControlLaw4 law4;
ControlLaw5 law5;
ControlLaw6 law6;
ControlLaw7 law7;
ControlLaw8 law8;

bool led_on;
bool debug = false;

void setup() {
    if (debug) Serial.begin(57600);
    if (debug) while(!Serial); // Wait for Opening Serial Monitor
    inputs.Setup();
    outputs.Setup();
    motors.Setup();
    randomSeed(analogRead(0));
    
    control_laws[0] = &lawHome;
    control_laws[1] = &law1;
    control_laws[2] = &law2;
    control_laws[3] = &law3;
    control_laws[4] = &law4;
    control_laws[5] = &law5;
    control_laws[6] = &law6;
    control_laws[7] = &law7;
    control_laws[8] = &law8;
  
    for (int i = 0; i< CONTROL_LAWS_SIZE; i++) {
      control_laws[i]->Setup();
    }
}

Status apply_control_loop(uint8_t control_law_index) {
  bool done = false;
  BaseControlLaw* control_law = control_laws[control_law_index];
  Status status;
  uint32_t step_count = 0;
  motors.SetPosPGain(0);     //set low torque at the beginning of any law execution
  while (!done) {
    //if(step_count<=68){motors.SetTorqueLimit(3+15*step_count);}     //------------- to debug quickly
    //if(step_count<=68){motors.SetCurrentLimit(3+15*step_count);}
    if(step_count<=40){motors.SetPosPGain(20*step_count);}
    //MotorArray current = motors.GetCurrentPositions();
    //for (int i =0; i<MotorArray::size; i++) {             //activating this, time_step will increase by 50%
    //Serial.println("pose: ");
    //Serial.println(current.values[i]);
    //}
    control_law->SetTimestep(step_count);
    //control_law->SetCurrentPoses(current);
    MotorArray next = control_law->GetNextTarget();
    motors.SetPositions(next);
    done = control_law->Done(status);
    step_count++;                                          // current time_step is about 2 ms
    if (debug) Serial.println(step_count+1000);
  }
  return status;
}

void loop() {
  // put your main code here, to run repeatedly:
  //uint8_t request = inputs.ReadInputStatus(1000);
  uint8_t request = 7;

  //if (request < 4) request = (request + random(0,4)) % 4;
  //if (request >= 4 && request < 7) request = 4 + ((request - 4 + random(0,3)) % 3);

  //for(int i=0;i<10;i++){
  while(1){
    //uint8_t request = random(0,CONTROL_LAWS_SIZE);
    //request = i;
    if (debug) Serial.println(request);
    Status status = apply_control_loop(request);
    //int32_t temp = motors.GetHighestTemperature();            // THIS WAY i can see the max temp and the Avg voltage
    //int32_t volt = motors.GetLowestVoltage();
    //if (temp > MAX_TEMP) status.SetHighTemperature();
    //if (volt < MIN_VOLT) status.SetLowBattery();
    //if (debug) Serial.println("highest temp: ");
    //if (debug) Serial.println(temp);
    //if (debug) Serial.println("and lowest voltage: ");
    //if (debug) Serial.println(volt);
    //outputs.WriteStatus(status);
    delay(2000);
  } 
  
    //led_on = !led_on;
    //digitalWrite(BOARD_LED_PIN, led_on?HIGH:LOW);
    //if (debug) Serial.println("no valid request found");
    //delay(2000);
    
}
