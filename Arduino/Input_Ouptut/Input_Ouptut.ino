/*
 * Author: Tyler Gragg
 * Date First Created: 01/18/2020
 * Project: AutoPinball
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// Flipper Outputs
#define LeftFlipper 20
#define RightFlipper 21

// Switch Input - Start button
#define StartButton 99 

// Switch Inputs - Playfield
// TOP
#define TopSwitch0 8  // Rollover
#define TopSwitch1 49 // Bumper
#define TopSwitch2 50 // Bumper
#define TopSwitch3 51 // Bumper
#define TopSwitch4 5  // Rollover
#define TopSwitch5 15 // Spinner

// MID
#define MidSwitch0 9  // Rollover
#define MidSwitch1 13 // Standup
#define MidSwitch2 7  // Rollover
#define MidSwitch3 14 // Standup
#define MidSwitch4 6  // Rollover
#define MidSwitch5 4  // Rollover

//BOT
#define BotSwitch0 10 // Rollover
#define BotSwitch1 11 // Rollover
#define BotSwitch2 53 // Slingshot
#define BotSwitch3 52 // Slingshot
#define BotSwitch4 3  // Rollover
#define BotSwitch5 2  // Rollover
#define BotSwitch6 18 // Flipper
#define BotSwitch7 19 // Flipper
#define BotSwitch8 12 // Rollover

// Light Outputs
// TOP
#define TopLight0 36
#define TopLight1 35
#define TopLight2 34

// MID
#define MidLight0 31
#define MidLight1 32
#define MidLight2 27
#define MidLight3 28
#define MidLight4 29
#define MidLight5 26
#define MidLight6 25
#define MidLight7 24

// BOT
#define BotLight0 33
#define BotLight1 30
#define BotLight2 23
#define BotLight3 22

// Coils
#define Coil0 44
#define Coil1 45
#define Coil2 46
#define Coil3 47
#define Coil4 48

// Last states
// TOP
uint8_t TopSwitch0Last = 0;
uint8_t TopSwitch1Last = 0;
uint8_t TopSwitch2Last = 0;
uint8_t TopSwitch3Last = 0;
uint8_t TopSwitch4Last = 0;
uint8_t TopSwitch5Last = 0;

// MID
uint8_t MidSwitch0Last = 0;
uint8_t MidSwitch1Last = 0;
uint8_t MidSwitch2Last = 0;
uint8_t MidSwitch3Last = 0;
uint8_t MidSwitch4Last = 0;
uint8_t MidSwitch5Last = 0;

// BOT
uint8_t BotSwitch0Last = 0;
uint8_t BotSwitch1Last = 0;
uint8_t BotSwitch2Last = 0;
uint8_t BotSwitch3Last = 0;
uint8_t BotSwitch4Last = 0;
uint8_t BotSwitch5Last = 0;
uint8_t BotSwitch6Last = 0;
uint8_t BotSwitch7Last = 0;
uint8_t BotSwitch8Last = 0;

uint8_t StartButtonLast = 0;

// ROS Node Handle
ros::NodeHandle nh;

// ROS Published messages
std_msgs::Bool empty_msg;

// TOP
ros::Publisher switch_top_0_pub("switch_top_0_triggered", &empty_msg);
ros::Publisher switch_top_1_pub("switch_top_1_triggered", &empty_msg);
ros::Publisher switch_top_2_pub("switch_top_2_triggered", &empty_msg);
ros::Publisher switch_top_3_pub("switch_top_3_triggered", &empty_msg);
ros::Publisher switch_top_4_pub("switch_top_4_triggered", &empty_msg);
ros::Publisher switch_top_5_pub("switch_top_5_triggered", &empty_msg);

// MID
ros::Publisher switch_mid_0_pub("switch_mid_0_triggered", &empty_msg);
ros::Publisher switch_mid_1_pub("switch_mid_1_triggered", &empty_msg);
ros::Publisher switch_mid_2_pub("switch_mid_2_triggered", &empty_msg);
ros::Publisher switch_mid_3_pub("switch_mid_3_triggered", &empty_msg);
ros::Publisher switch_mid_4_pub("switch_mid_4_triggered", &empty_msg);
ros::Publisher switch_mid_5_pub("switch_mid_5_triggered", &empty_msg);

// BOT
ros::Publisher switch_bot_0_pub("switch_bot_0_triggered", &empty_msg);
ros::Publisher switch_bot_1_pub("switch_bot_1_triggered", &empty_msg);
ros::Publisher switch_bot_2_pub("switch_bot_2_triggered", &empty_msg);
ros::Publisher switch_bot_3_pub("switch_bot_3_triggered", &empty_msg);
ros::Publisher switch_bot_4_pub("switch_bot_4_triggered", &empty_msg);
ros::Publisher switch_bot_5_pub("switch_bot_5_triggered", &empty_msg);
ros::Publisher switch_bot_6_pub("switch_bot_6_triggered", &empty_msg);
ros::Publisher switch_bot_7_pub("switch_bot_7_triggered", &empty_msg);
ros::Publisher switch_bot_8_pub("switch_bot_8_triggered", &empty_msg);

ros::Publisher start_button_pub("switch_start_button_triggered", &empty_msg);

//TODO: Possibly add callback for each flipper - then I could change to bool
void flip_callback(const std_msgs::Int32& flipper){
  // 1 = left_flipper ON, 2 = right_flipper ON, -1 = left_flipper OFF, -2 = right_flipper OFF
  if (flipper.data == 1){
    digitalWrite(LeftFlipper, HIGH);
  }
  else if(flipper.data == 2){
    digitalWrite(RightFlipper, HIGH); 
  }
  else if(flipper.data == -1){
    digitalWrite(LeftFlipper, LOW);
  }
  else if(flipper.data == -2){
    digitalWrite(RightFlipper, LOW);
  }
}

// Turn on whatever int comes in
void pin_on_callback(const std_msgs::Int32& pin){
  digitalWrite(pin.data, HIGH);
}

// Turn off whatever int comes in
void pin_off_callback(const std_msgs::Int32& pin){
  digitalWrite(pin.data, LOW);
}


// ROS Subscibed messages
ros::Subscriber<std_msgs::Int32> flip_sub("flip_flipper", &flip_callback);
ros::Subscriber<std_msgs::Int32> pin_on_sub("pin_on", &pin_on_callback);
ros::Subscriber<std_msgs::Int32> pin_off_sub("pin_off", &pin_off_callback);

// Check all switches, then publish if they are triggered
void checkSwitches(){
  //TOP
  uint8_t curTopSwitch0 = digitalRead(TopSwitch0);
  uint8_t curTopSwitch1 = digitalRead(TopSwitch1);
  uint8_t curTopSwitch2 = digitalRead(TopSwitch2);
  uint8_t curTopSwitch3 = digitalRead(TopSwitch3);
  uint8_t curTopSwitch4 = digitalRead(TopSwitch4);
  uint8_t curTopSwitch5 = digitalRead(TopSwitch5);

  //MID
  uint8_t curMidSwitch0 = digitalRead(MidSwitch0);
  uint8_t curMidSwitch1 = digitalRead(MidSwitch1);
  uint8_t curMidSwitch2 = digitalRead(MidSwitch2);
  uint8_t curMidSwitch3 = digitalRead(MidSwitch3);
  uint8_t curMidSwitch4 = digitalRead(MidSwitch4);
  uint8_t curMidSwitch5 = digitalRead(MidSwitch5);
  
  //BOT
  uint8_t curBotSwitch0 = digitalRead(BotSwitch0);
  uint8_t curBotSwitch1 = digitalRead(BotSwitch1);
  uint8_t curBotSwitch2 = digitalRead(BotSwitch2);
  uint8_t curBotSwitch3 = digitalRead(BotSwitch3);
  uint8_t curBotSwitch4 = digitalRead(BotSwitch4);
  uint8_t curBotSwitch5 = digitalRead(BotSwitch5);
  uint8_t curBotSwitch6 = digitalRead(BotSwitch6);
  uint8_t curBotSwitch7 = digitalRead(BotSwitch7);
  uint8_t curBotSwitch8 = digitalRead(BotSwitch8);

  uint8_t curStartButton = digitalRead(StartButton);
  
  // -----------  TOP  -----------
  if (!curTopSwitch0 && curTopSwitch0 != TopSwitch0Last){
    switch_top_0_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curTopSwitch1 && curTopSwitch1 != TopSwitch1Last){
    switch_top_1_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curTopSwitch2 && curTopSwitch2 != TopSwitch2Last){
    switch_top_2_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curTopSwitch3 && curTopSwitch3 != TopSwitch3Last){
    switch_top_3_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curTopSwitch4 && curTopSwitch4 != TopSwitch4Last){
    switch_top_4_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curTopSwitch5 && curTopSwitch5 != TopSwitch5Last){
    switch_top_5_pub.publish(&empty_msg);
    nh.spinOnce();
  }

  // -----------  MID  -----------
  if (!curMidSwitch0 && curMidSwitch0 != MidSwitch0Last){
    switch_mid_0_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curMidSwitch1 && curMidSwitch1 != MidSwitch1Last){
    switch_mid_1_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curMidSwitch2 && curMidSwitch2 != MidSwitch2Last){
    switch_mid_2_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curMidSwitch3 && curMidSwitch3 != MidSwitch3Last){
    switch_mid_3_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curMidSwitch4 && curMidSwitch4 != MidSwitch4Last){
    switch_mid_4_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curMidSwitch5 && curMidSwitch5 != MidSwitch5Last){
    switch_mid_5_pub.publish(&empty_msg);
    nh.spinOnce();
  }

  // -----------  BOT  -----------
  if (!curBotSwitch0 && curBotSwitch0 != BotSwitch0Last){
    switch_bot_0_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curBotSwitch1 && curBotSwitch1 != BotSwitch1Last){
    switch_bot_1_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curBotSwitch2 && curBotSwitch2 != BotSwitch2Last){
    switch_bot_2_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curBotSwitch3 && curBotSwitch3 != BotSwitch3Last){
    switch_bot_3_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curBotSwitch4 && curBotSwitch4 != BotSwitch4Last){
    switch_bot_4_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curBotSwitch5 && curBotSwitch5 != BotSwitch5Last){
    switch_bot_5_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (curBotSwitch6 && curBotSwitch6 != BotSwitch6Last){ // FLIPPER - different logic
    switch_bot_6_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (curBotSwitch7 && curBotSwitch7 != BotSwitch7Last){ // FLIPPER - different logic
    switch_bot_7_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curBotSwitch8 && curBotSwitch8 != BotSwitch8Last){
    switch_bot_8_pub.publish(&empty_msg);
    nh.spinOnce();
  }

  // -----------  OTHER  -----------
  if (!curStartButton && curStartButton != StartButtonLast){
    start_button_pub.publish(&empty_msg);
    nh.spinOnce();
  }

  //TOP
  TopSwitch0Last = curTopSwitch0;
  TopSwitch1Last = curTopSwitch1;
  TopSwitch2Last = curTopSwitch2;
  TopSwitch3Last = curTopSwitch3;
  TopSwitch4Last = curTopSwitch4;
  TopSwitch5Last = curTopSwitch5;
    
  //MID
  MidSwitch0Last = curMidSwitch0;
  MidSwitch1Last = curMidSwitch1;
  MidSwitch2Last = curMidSwitch2;
  MidSwitch3Last = curMidSwitch3;
  MidSwitch4Last = curMidSwitch4;
  MidSwitch5Last = curMidSwitch5;
  
  //BOT
  BotSwitch0Last = curBotSwitch0;
  BotSwitch1Last = curBotSwitch1;
  BotSwitch2Last = curBotSwitch2;
  BotSwitch3Last = curBotSwitch3;
  BotSwitch4Last = curBotSwitch4;
  BotSwitch5Last = curBotSwitch5;
  BotSwitch6Last = curBotSwitch6;
  BotSwitch7Last = curBotSwitch7;
  BotSwitch8Last = curBotSwitch8;

  //OTHER
  StartButtonLast = curStartButton;
}

void setup(){
  //Flippers
  pinMode(LeftFlipper, OUTPUT);
  pinMode(RightFlipper, OUTPUT);
  
  // Start Button
  pinMode(StartButton, INPUT_PULLUP);
  
  // Switches
  //TOP
  pinMode(TopSwitch0, INPUT_PULLUP);
  pinMode(TopSwitch1, INPUT_PULLUP);
  pinMode(TopSwitch2, INPUT_PULLUP);
  pinMode(TopSwitch3, INPUT_PULLUP);
  pinMode(TopSwitch4, INPUT_PULLUP);
  pinMode(TopSwitch5, INPUT_PULLUP);
  
  //MID
  pinMode(MidSwitch0, INPUT_PULLUP);
  pinMode(MidSwitch1, INPUT_PULLUP);
  pinMode(MidSwitch2, INPUT_PULLUP);
  pinMode(MidSwitch3, INPUT_PULLUP);
  pinMode(MidSwitch4, INPUT_PULLUP);
  pinMode(MidSwitch5, INPUT_PULLUP);
  
  //BOT
  pinMode(BotSwitch0, INPUT_PULLUP);
  pinMode(BotSwitch1, INPUT_PULLUP);
  pinMode(BotSwitch2, INPUT_PULLUP);
  pinMode(BotSwitch3, INPUT_PULLUP);
  pinMode(BotSwitch4, INPUT_PULLUP);
  pinMode(BotSwitch5, INPUT_PULLUP);
  pinMode(BotSwitch6, INPUT); // FLIPPER - different logic
  pinMode(BotSwitch7, INPUT); // FLIPPER - different logic
  pinMode(BotSwitch8, INPUT_PULLUP);
  
  // Lights
  //TOP
  pinMode(TopLight0, OUTPUT);
  pinMode(TopLight1, OUTPUT);
  pinMode(TopLight2, OUTPUT);
  
  //MID
  pinMode(MidLight0, OUTPUT);
  pinMode(MidLight1, OUTPUT);
  pinMode(MidLight2, OUTPUT);
  pinMode(MidLight3, OUTPUT);
  pinMode(MidLight4, OUTPUT);
  pinMode(MidLight5, OUTPUT);
  pinMode(MidLight6, OUTPUT);
  pinMode(MidLight7, OUTPUT);
  
  //BOT
  pinMode(BotLight0, OUTPUT);
  pinMode(BotLight1, OUTPUT);
  pinMode(BotLight2, OUTPUT);
  pinMode(BotLight3, OUTPUT);

  //Coils
  pinMode(Coil0, OUTPUT);
  pinMode(Coil1, OUTPUT);
  pinMode(Coil2, OUTPUT);
  pinMode(Coil3, OUTPUT);
  pinMode(Coil4, OUTPUT);

  // Set everything low to start out
  digitalWrite(LeftFlipper, LOW);
  digitalWrite(RightFlipper, LOW);
  digitalWrite(TopLight0, LOW);
  digitalWrite(TopLight1, LOW);
  digitalWrite(TopLight2, LOW);
  digitalWrite(MidLight0, LOW);
  digitalWrite(MidLight1, LOW);
  digitalWrite(MidLight2, LOW);
  digitalWrite(MidLight3, LOW);
  digitalWrite(MidLight4, LOW);
  digitalWrite(MidLight5, LOW);
  digitalWrite(MidLight6, LOW);
  digitalWrite(MidLight7, LOW);
  digitalWrite(BotLight0, LOW);
  digitalWrite(BotLight1, LOW);
  digitalWrite(BotLight2, LOW);
  digitalWrite(BotLight3, LOW);
  digitalWrite(Coil0, LOW);
  digitalWrite(Coil1, LOW);
  digitalWrite(Coil2, LOW);
  digitalWrite(Coil3, LOW);
  digitalWrite(Coil4, LOW);
  
  // Initialize ROS stuff
  nh.initNode();
  
  //TOP
  nh.advertise(switch_top_0_pub);
  nh.advertise(switch_top_1_pub);
  nh.advertise(switch_top_2_pub);
  nh.advertise(switch_top_3_pub);
  nh.advertise(switch_top_4_pub);
  nh.advertise(switch_top_5_pub);

  //MID
  nh.advertise(switch_mid_0_pub);
  nh.advertise(switch_mid_1_pub);
  nh.advertise(switch_mid_2_pub);
  nh.advertise(switch_mid_3_pub);
  nh.advertise(switch_mid_4_pub);
  nh.advertise(switch_mid_5_pub);
  
  //BOT
  nh.advertise(switch_bot_0_pub);
  nh.advertise(switch_bot_1_pub);
  nh.advertise(switch_bot_2_pub);
  nh.advertise(switch_bot_3_pub);
  nh.advertise(switch_bot_4_pub);
  nh.advertise(switch_bot_5_pub);
  nh.advertise(switch_bot_6_pub);
  nh.advertise(switch_bot_7_pub);
  nh.advertise(switch_bot_8_pub);

  //OTHER
  nh.advertise(start_button_pub);
  
  // Subscribers
  nh.subscribe(flip_sub);
  nh.subscribe(pin_on_sub);
  nh.subscribe(pin_off_sub);

  // Boolean empty message
  empty_msg.data = true;
}

void loop(){
  checkSwitches();
  nh.spinOnce();
}
