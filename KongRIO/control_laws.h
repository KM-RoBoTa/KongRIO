#include "motor_handler.h" //for joint_value
#include "motors_manager.h" //for NUM_MOTORS, MotorArray
#include "input_manager.h" //for Status

#define CONTROL_LAWS_SIZE 9 //this should be 7 if the input manager is used

class BaseControlLaw {
protected:
  virtual void Init() = 0;
  uint32_t steps_;
  MotorArray current_poses_;
  bool done_ = false;
  Status status_;
  uint8_t scanned_id[NUM_MOTORS] = {1,2,3,4,5,6,7,8}; //all motors ID's in the robot

public:
  void Setup() {
    steps_ = 0;
    current_poses_.reset();
    done_ = false;
    status_.ClearAll();
    Init();

  }
  void SetTimestep(uint32_t elapsed_steps) {this->steps_ = elapsed_steps;}
  void SetCurrentPoses(MotorArray current_poses) {this->current_poses_ = current_poses;}
    
  virtual MotorArray GetNextTarget() = 0;
  bool Done(Status& final_status) {
    if (done_) {
      final_status = status_;
    }
    return done_;
  }
};

/**
 * List of the 10 control laws
 */
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//ControlLaw for the Homing motion
class ControlLaw0 : public BaseControlLaw {
protected:
  virtual void Init() {
    // Whatever needed to init for this specific control law
  }  

public:
  virtual MotorArray GetNextTarget() {
    MotorArray result;
    result.reset();
    float ini_pose[] = {2048,2048,2048,2048,2048,2048,2048,2048};
    float home_steps = 300;
    float time_step = steps_-home_steps;
    
    if (steps_<home_steps){
      
      for (int i=0; i < MotorArray::size; i++){
        result.values[i]=ini_pose[i];
      }
    }

    else {
      
      for (int i=0; i < MotorArray::size; i++){
        result.values[i] = ini_pose[i];
      }
      
      status_.ClearAll(); //status is clear before setting done_
      if (steps_>home_steps+3000)  // time_step is in the order of 2 ms -> check .ino file
      done_ = true; // all done, the function will no longer be called      
    }
    return result;
  }
};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ControlLaw for the swim 1

class ControlLaw1 : public BaseControlLaw {
protected:
  virtual void Init() {
    // Whatever needed to init for this specific control law
    // Definition of gait parameters. 60deg=200ticks 40deg=135 30deg=102 20deg=68 10deg=34
    a = 114;        //amplitude modules 10.
    o = 2048;        //offset modules. 2048 ticks equivalent to 0 degree (center)
    wlt = 2*PI/8;  //wavelength
    frq = PI/1000;    //frequency -> PI/1000 with 16000 timesteps makes 8 waves in 20 seconds. Make the math.
  }  
  float a, o, wlt, frq;
 
public:
  virtual MotorArray GetNextTarget() {
    MotorArray result;
    result.reset();
    for (int i=0; i < MotorArray::size; i++) {
      result.values[i] = o + a*sin(-wlt*(i+1) + frq*steps_);
      
      status_.ClearAll(); //status is clear before setting done_
      if (steps_>16000)  // time_step is in the order of 2 ms -> check .ino file
      done_ = true; // all done, the function will no longer be called
    }
    return result;
  }
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ControlLaw for the swim 2

class ControlLaw2 : public BaseControlLaw {
protected:
  virtual void Init() {
    // Whatever needed to init for this specific control law
    // Definition of gait parameters. 60deg=200ticks 40deg=135 30deg=102 20deg=68 10deg=34
    a = 227;        //amplitude modules 20.
    o = 2048;        //offset modules. 2048 ticks equivalent to 0 degree (center)
    wlt = 2*PI/8;  //wavelength
    frq = PI/1000;    //frequency -> PI/1000 with 16000 timesteps makes 8 waves in 20 seconds. Make the math.
  }  
  float a, o, wlt, frq;
 
public:
  virtual MotorArray GetNextTarget() {
    MotorArray result;
    result.reset();
    for (int i=0; i < MotorArray::size; i++) {
      result.values[i] = o + a*sin(-wlt*(i+1) + frq*steps_);
      
      status_.ClearAll(); //status is clear before setting done_
      if (steps_>16000)  // time_step is in the order of 2 ms -> check .ino file
      done_ = true; // all done, the function will no longer be called
    }
    return result;
  }
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ControlLaw for the swim 3

class ControlLaw3 : public BaseControlLaw {
protected:
  virtual void Init() {
    // Whatever needed to init for this specific control law
    // Definition of gait parameters. 60deg=200ticks 40deg=135 30deg=102 20deg=68 10deg=34
    a = 341;        //amplitude modules 30.
    o = 2048;        //offset modules. 2048 ticks equivalent to 0 degree (center)
    wlt = 2*PI/8;  //wavelength
    frq = PI/1000;    //frequency -> PI/1000 with 16000 timesteps makes 8 waves in 20 seconds. Make the math.
  }  
  float a, o, wlt, frq;
 
public:
  virtual MotorArray GetNextTarget() {
    MotorArray result;
    result.reset();
    for (int i=0; i < MotorArray::size; i++) {
      result.values[i] = o + a*sin(-wlt*(i+1) + frq*steps_);
      
      status_.ClearAll(); //status is clear before setting done_
      if (steps_>16000)  // time_step is in the order of 2 ms -> check .ino file
      done_ = true; // all done, the function will no longer be called
    }
    return result;
  }
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ControlLaw for the swim 4

class ControlLaw4 : public BaseControlLaw {
protected:
  virtual void Init() {
    // Whatever needed to init for this specific control law
    // Definition of gait parameters. 60deg=200ticks 40deg=135 30deg=102 20deg=68 10deg=34
    a = 114;        //amplitude modules 10.
    o = 2048;        //offset modules. 2048 ticks equivalent to 0 degree (center)
   wlt = 2*PI/12;  //wavelength
    frq = PI/1000;    //frequency -> PI/1000 with 16000 timesteps makes 8 waves in 20 seconds. Make the math.
  }  
  float a, o, wlt, frq;
 
public:
  virtual MotorArray GetNextTarget() {
    MotorArray result;
    result.reset();
    for (int i=0; i < MotorArray::size; i++) {
      result.values[i] = o + a*sin(-wlt*(i+1) + frq*steps_);
      
      status_.ClearAll(); //status is clear before setting done_
      if (steps_>16000)  // time_step is in the order of 2 ms -> check .ino file
      done_ = true; // all done, the function will no longer be called
    }
    return result;
  }
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ControlLaw for the swim 5

class ControlLaw5 : public BaseControlLaw {
protected:
  virtual void Init() {
    // Whatever needed to init for this specific control law
    // Definition of gait parameters. 60deg=200ticks 40deg=135 30deg=102 20deg=68 10deg=34
    a = 227;        //amplitude modules 20.
    o = 2048;        //offset modules. 2048 ticks equivalent to 0 degree (center)
    wlt = 2*PI/12;  //wavelength
    frq = PI/1000;    //frequency -> PI/1000 with 16000 timesteps makes 8 waves in 20 seconds. Make the math.
  }  
  float a, o, wlt, frq;
 
public:
  virtual MotorArray GetNextTarget() {
    MotorArray result;
    result.reset();
    for (int i=0; i < MotorArray::size; i++) {
      result.values[i] = o + a*sin(-wlt*(i+1) + frq*steps_);
      
      status_.ClearAll(); //status is clear before setting done_
      if (steps_>16000)  // time_step is in the order of 2 ms -> check .ino file
      done_ = true; // all done, the function will no longer be called
    }
    return result;
  }
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ControlLaw for the swim 6

class ControlLaw6 : public BaseControlLaw {
protected:
  virtual void Init() {
    // Whatever needed to init for this specific control law
    // Definition of gait parameters. 60deg=200ticks 40deg=135 30deg=102 20deg=68 10deg=34
    a = 341;        //amplitude modules 30.
    o = 2048;        //offset modules. 2048 ticks equivalent to 0 degree (center)
    wlt = 2*PI/12;  //wavelength
    frq = PI/1000;    //frequency -> PI/1000 with 16000 timesteps makes 8 waves in 20 seconds. Make the math.
  }  
  float a, o, wlt, frq;
 
public:
  virtual MotorArray GetNextTarget() {
    MotorArray result;
    result.reset();
    for (int i=0; i < MotorArray::size; i++) {
      result.values[i] = o + a*sin(-wlt*(i+1) + frq*steps_);
      
      status_.ClearAll(); //status is clear before setting done_
      if (steps_>16000)  // time_step is in the order of 2 ms -> check .ino file
      done_ = true; // all done, the function will no longer be called
    }
    return result;
  }
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ControlLaw for the swim 7

class ControlLaw7 : public BaseControlLaw {
protected:
  virtual void Init() {
    // Whatever needed to init for this specific control law
    // Definition of gait parameters. 60deg=200ticks 40deg=135 30deg=102 20deg=68 10deg=34
    a = 341;        //amplitude modules 30.
    o = 2048;        //offset modules. 2048 ticks equivalent to 0 degree (center)
    wlt = 2*PI/8;  //wavelength
    frq = PI/250;    //frequency -> PI/1000 with 16000 timesteps makes 8 waves in 20 seconds. Make the math.
  }  
  float a, o, wlt, frq;
 
public:
  virtual MotorArray GetNextTarget() {
    MotorArray result;
    result.reset();
    for (int i=0; i < MotorArray::size; i++) {
      a=114+(341/8*(i));
      result.values[i] = o + a*sin(-wlt*(i+1) + frq*steps_);
      
      status_.ClearAll(); //status is clear before setting done_
      if (steps_>16000)  // time_step is in the order of 2 ms -> check .ino file
      done_ = true; // all done, the function will no longer be called
    }
    return result;
  }
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ControlLaw for the swim 8

class ControlLaw8 : public BaseControlLaw {
protected:
  virtual void Init() {
    // Whatever needed to init for this specific control law
    // Definition of gait parameters. 60deg=200ticks 40deg=135 30deg=102 20deg=68 10deg=34
    a = 341;        //amplitude modules 30.
    o = 2048;        //offset modules. 2048 ticks equivalent to 0 degree (center)
    wlt = 2*PI/8;  //wavelength
    frq = PI/500;    //frequency -> PI/1000 with 16000 timesteps makes 8 waves in 20 seconds. Make the math.
  }  
  float a, o, wlt, frq;
 
public:
  virtual MotorArray GetNextTarget() {
    MotorArray result;
    result.reset();
    for (int i=0; i < MotorArray::size; i++) {
      a=114+(341/8*(i));
      result.values[i] = o + a*sin(-wlt*(i+1) + frq*steps_);
      
      status_.ClearAll(); //status is clear before setting done_
      if (steps_>16000)  // time_step is in the order of 2 ms -> check .ino file
      done_ = true; // all done, the function will no longer be called
    }
    return result;
  }
};
