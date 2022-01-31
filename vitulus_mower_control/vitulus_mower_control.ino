#include <TMC2130Stepper.h>
#include <AccelStepper.h>
#include <AutoPID.h>
#include <ros.h>
#include <std_msgs/Int32.h>


ros::NodeHandle  nh;

std_msgs::Int32 rpm_msg;
ros::Publisher mower_rpm("mower_rpm", &rpm_msg);

std_msgs::Int32 motor_on_msg;
ros::Publisher mower_motor_on("mower_motor_on", &motor_on_msg);

std_msgs::Int32 height_msg;
ros::Publisher mower_cut_height("mower_cut_height", &height_msg);
//
//std_msgs::Int32 mower_msg;
//ros::Publisher mower_message("mower_message", &mower_msg);


bool endstop = false;
bool gohome = true;
bool switchstate = true;
int  motor_on = 0;
int  cut_height = 0;


#define PWM_R 5
#define PWM_L 6
double countb = 0;
double count;
unsigned long lastmillis;
unsigned long lastmillisloop;
double outputVal, setPoint;


#define EN_PIN    7  
#define DIR_PIN   8  
#define STEP_PIN  9  
#define CS_PIN    10  
#define MOSI_PIN  11
#define MISO_PIN 12
#define SCK_PIN  13
constexpr uint32_t steps_per_mm = 80*15;

#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 2.7    // 1.7
#define KI 5    // 25
#define KD 2.2  // 0.02

#define INTERVAL 5

//unsigned long interval = 5;
unsigned long rpm = 3300;
unsigned long ppr = 256;
unsigned long ms = 60000/INTERVAL;

#define SWITCH_PIN  3


TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

AutoPID myPID(&count, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setMotor( const std_msgs::Int32& set_motor_msg);
void setRpm( const std_msgs::Int32& set_rpm_msg);
void setHeight( const std_msgs::Int32& set_height_msg);
ros::Subscriber<std_msgs::Int32> sub_motor("mower_set_motor", setMotor );
ros::Subscriber<std_msgs::Int32> sub_rpm("mower_set_rpm", setRpm );
ros::Subscriber<std_msgs::Int32> sub_height("mower_set_height", setHeight );


void setup() {

    nh.initNode();
    nh.advertise(mower_rpm);
    nh.advertise(mower_motor_on);
    nh.advertise(mower_cut_height);
//    nh.advertise(mower_message);
    nh.subscribe(sub_motor);
    nh.subscribe(sub_rpm);
    nh.subscribe(sub_height);
    
//    attachInterrupt(1, switchbtn, LOW);
//    attachInterrupt(1, switchbtnup, FALLING);
    SPI.begin();
    Serial.begin(115200);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    driver.begin();             
    driver.rms_current(600);    
    driver.stealthChop(0);   
    driver.stealth_autoscale(1);
    driver.microsteps(4);

    stepper.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
    stepper.setAcceleration(100*steps_per_mm); // 2000mm/s^2
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);

    
    pinMode(PWM_R, OUTPUT);
    pinMode(PWM_L, OUTPUT);
    analogWrite(PWM_R, 0);
    analogWrite(PWM_L, 0);
    setPoint = double(rpm*ppr/ms); //205-3000rpm 4096
    attachInterrupt(0, prerusenib, RISING);
    lastmillis = millis();
    myPID.setTimeStep(INTERVAL);

    pinMode(SWITCH_PIN, INPUT);
    if (digitalRead(SWITCH_PIN) == false)
    {
      endstop = true;
      gohome = false;
      switchstate = false;
    }else{
      switchstate = true;
    }


}

void loop() {
//  Serial.println(gohome);

  

  
  if ((millis() - lastmillis) >= 1000){
    rpm_msg.data = (count/ppr*ms);
    mower_rpm.publish( &rpm_msg );
    motor_on_msg.data = motor_on;
    mower_motor_on.publish( &motor_on_msg );
    height_msg.data = (stepper.currentPosition()/steps_per_mm);
    mower_cut_height.publish( &height_msg );
//
//    mower_msg.data = (count/ppr*ms) * 1000;
//    mower_msg.data = (mower_msg.data + (motor_on)) * 1000;
//    mower_msg.data = (mower_msg.data + (stepper.currentPosition()/steps_per_mm));
    
//    mower_message.publish( &mower_msg );
    
    nh.spinOnce();

    lastmillis = millis();

  }


  // PID MOWER MOTOR
  if ((millis() - lastmillisloop) >= INTERVAL){
    lastmillisloop = millis();
    count = countb;
    countb = 0;
    if (motor_on){
      myPID.run();
      analogWrite(PWM_L, outputVal);
    }else{
      myPID.stop();
      analogWrite(PWM_L, 0);
    }
    
    
  }

  // STEPPER
  if (digitalRead(SWITCH_PIN) == false)
    {
//      Serial.println("ENDSTOP PRESSED");
      if (switchstate == true){
        endstop = true;
        gohome = false;
        stepper.stop();
        stepper.move(0*steps_per_mm);
        stepper.setCurrentPosition(0);
        stepper.disableOutputs();
      }
      if (endstop == true){
      stepper.moveTo(100*steps_per_mm); // Move 100mm
      stepper.enableOutputs();
//      Serial.println("ENDSTOP PRESSED1");
      gohome = false;
//      Serial.println("ON");
      }
      switchstate = false;
    }else{
//      Serial.println("ENDSTOP RELEASED");
//      Serial.println("OFF");
      if (endstop == true){
        endstop = false;

        stepper.stop();
        stepper.move(0*steps_per_mm);
        stepper.disableOutputs();
        stepper.setCurrentPosition(0);
//        Serial.println("ZERO LEVEL");
//        Serial.println("OFF END STOP");
        
      }
      switchstate = true;
    }
  
 
    
    if (gohome == true){
      stepper.moveTo(-100*steps_per_mm); // Move 100mm
      stepper.enableOutputs();
//      Serial.println("GOING HOME1");
    }
    
    if (stepper.distanceToGo() == 0) {
        stepper.disableOutputs();
    }
    
    stepper.run();
    

}

void prerusenib() {
  countb ++;
}

//void switchbtn(){
//  endstop = false;
//  gohome = false;
//  zero = true;
//  stepper.move(2*steps_per_mm); // Move 100mm
//  stepper.enableOutputs();
//  Serial.println("ENDSTOP PRESSED");
////  Serial.println("ENDSTOP PRESSED");
//}



void setMotor( const std_msgs::Int32& set_motor_msg){
  motor_on = set_motor_msg.data; 
}

void setRpm( const std_msgs::Int32& set_rpm_msg){
  setPoint = (set_rpm_msg.data*ppr/ms); 
}

void setHeight( const std_msgs::Int32& set_height_msg){ 
  stepper.enableOutputs();
  stepper.moveTo(set_height_msg.data*steps_per_mm);
}
