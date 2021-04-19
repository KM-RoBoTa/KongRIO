#ifndef INPUT_MANAGER
#define INPUT_MANAGER

#include <OpenCM904.h>

#define MAX_TEMP 65
#define MIN_VOLT 106
/**
 * Represents the motor status, currently no idea how to read this from motors
 */
struct Status{
  Status(): raw_data(0) {}
  Status(uint8_t status): raw_data(status){}
  void ClearAll(){raw_data = 0;}
  void SetError(){raw_data |= 0x1;}
  void ClearError(){raw_data &= (0xFF-0x1);}
  void SetLowBattery(){raw_data |= 0x2;}
  void SetBatteryOk(){raw_data &= (0xFF - 0x2);}
  void SetHighTemperature(){raw_data |= 0x4;}
  void SetTemperatureOk(){raw_data &= (0xFF - 0x4);}
  uint8_t raw_data = 0;
};

class InputManager {
  const int SYNC_PIN = 16;
  #define INPUT_MANAGER_SIZE 3
  const int DATA_PINS[INPUT_MANAGER_SIZE] = {19,18,17};
public:

  void Setup(){
    for (int i = 0; i< INPUT_MANAGER_SIZE; i++) {  
      pinMode(DATA_PINS[i], INPUT_PULLDOWN);  //pulldown: we expect a pullup on the other side
    }
    pinMode(SYNC_PIN, INPUT_PULLDOWN);
  }

  /**
   * Returns a value between 0 and 7 if there was a reading, 0xFF if no reading could be found
   * @param timeout (ms) the time to wait for the input sync sequence before returning 0xFF
   * 
   * Note: this is a blocking call until an input is available or the timeout expires
   */
  uint8_t ReadInputStatus(uint32_t timeout_ms) {                              //Kamilo: Explain Mirko about the 20s of max ON by Trigger board
    //let's check N times
    uint32_t sleep_time = 1 + (timeout_ms < 500 ? timeout_ms/10 : 50);
    int counter = 1 + timeout_ms / sleep_time;
    while (counter > 0) {
      if (digitalRead(SYNC_PIN)) {
        uint8_t status = 0;
        for (int i = 0; i< INPUT_MANAGER_SIZE; i++) {
          status = status + (digitalRead(DATA_PINS[i])<<i);
        }
        return status;
      }
      delay(sleep_time);
      counter--;
    }
  return 0xFF;    
  }
};


class OutputManager {
  const int SYNC_PIN = 6;
  #define OUTPUT_MANAGER_SIZE 3
  const int DATA_PINS[OUTPUT_MANAGER_SIZE] = {9,8,7};

  const int sync_delay_ms = 50;
  const int data_delay_ms = 400;

  public:
  
  void Setup(){
    for (int i = 0; i< OUTPUT_MANAGER_SIZE; i++) {    
      pinMode(DATA_PINS[i], OUTPUT);
    }
    pinMode(SYNC_PIN, OUTPUT);
    pinMode(BOARD_LED_PIN, OUTPUT);
  }

  /**
   * Fulfills the protocol requirement for writing data over the Err
   */
  void WriteStatus(Status status){
    for (int i = 0; i< OUTPUT_MANAGER_SIZE; i++) { 
      digitalWrite(DATA_PINS[i],  ((status.raw_data >> i) & 0x1));
    }
    delay(sync_delay_ms);
    digitalWrite(SYNC_PIN, HIGH);
    delay(data_delay_ms);
    digitalWrite(SYNC_PIN, LOW);
    delay(10);
  }
}; 

#endif
