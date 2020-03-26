/*
 * Author: Tyler Gragg
 * Date First Created: 01/18/2019
 * Project: AutoPinball
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// Light Outputs
#define TopLight0 99
#define TopLight1 99
#define MidLight0 99
#define MidLight1 99
#define BotLight0 99
#define BotLight1 99

// Flipper Outputs (writing to flippers)
#define LeftFlipperOutput 40
#define RightFlipperOutput 41

// Flipper Inputs (reading from flipper status)
#define LeftFlipperInput 99
#define RightFlipperInput 99

// Switch Inputs - Playfield
#define TopSwitch0 99
#define TopSwitch1 99
#define MidSwitch0 38 // Right Slingshot
#define MidSwitch1 99
#define BotSwitch0 39 // Left Slingshot
#define BotSwitch1 99

// Switch Input - Start button
#define StartButton 99 

uint8_t TopSwitch0Last = 0;
uint8_t TopSwitch1Last = 0;
uint8_t MidSwitch0Last = 0;
uint8_t MidSwitch1Last = 0;
uint8_t BotSwitch0Last = 0;
uint8_t BotSwitch1Last = 0;

uint8_t StartButtonLast = 0;

uint8_t LeftFlipperInputLast = 0;
uint8_t RightFlipperInputLast = 0;

// ROS Node Handle
ros::NodeHandle nh;

// ROS Published messages
std_msgs::Bool empty_msg;

ros::Publisher switch_top_0_pub("switch_top_0_triggered", &empty_msg);
ros::Publisher switch_top_1_pub("switch_top_1_triggered", &empty_msg);
ros::Publisher switch_mid_0_pub("switch_mid_0_triggered", &empty_msg);
ros::Publisher switch_mid_1_pub("switch_mid_1_triggered", &empty_msg);
ros::Publisher switch_bot_0_pub("switch_bot_0_triggered", &empty_msg);
ros::Publisher switch_bot_1_pub("switch_bot_1_triggered", &empty_msg);

ros::Publisher start_button_pub("switch_start_button_triggered", &empty_msg);

ros::Publisher left_flipper_input_pub("left_flipper_input_triggered", &empty_msg);
ros::Publisher right_flipper_input_pub("right_flipper_input_triggered", &empty_msg);

//TODO: Possibly add callback for each flipper - then I could change to bool
void flip_callback(const std_msgs::Int32& flipper){
  // 1 = left_flipper ON, 2 = right_flipper ON, -1 = left_flipper OFF, -2 = right_flipper OFF
  if (flipper.data == 1){
    digitalWrite(LeftFlipperOutput, HIGH);
  }
  else if(flipper.data == 2){
    digitalWrite(RightFlipperOutput, HIGH);
  }
  else if(flipper.data == -1){
    digitalWrite(LeftFlipperOutput, LOW);
  }
  else if(flipper.data == -2){
    digitalWrite(RightFlipperOutput, LOW);
  }
}

// Turn on whatever int comes in
void light_on_callback(const std_msgs::Int32& light){
  digitalWrite(light.data, HIGH);
}

// Turn off whatever int comes in
void light_off_callback(const std_msgs::Int32& light){
  digitalWrite(light.data, LOW);
}


// ROS Subscibed messages
ros::Subscriber<std_msgs::Int32> flip_sub("flip_flipper", &flip_callback);
ros::Subscriber<std_msgs::Int32> light_on_sub("light_on", &light_on_callback);
ros::Subscriber<std_msgs::Int32> light_off_sub("light_off", &light_off_callback);

// Check all switches, then publish if they are triggered
void checkSwitches(){
  uint8_t curTopSwitch0 = digitalRead(TopSwitch0);
  uint8_t curTopSwitch1 = digitalRead(TopSwitch1);
  uint8_t curMidSwitch0 = digitalRead(MidSwitch0);
  uint8_t curMidSwitch1 = digitalRead(MidSwitch1);
  uint8_t curBotSwitch0 = digitalRead(BotSwitch0);
  uint8_t curBotSwitch1 = digitalRead(BotSwitch1);

  uint8_t curStartButton = digitalRead(StartButton);

  uint8_t curLeftFlipperInput = digitalRead(LeftFlipperInput);
  uint8_t curRightFlipperInput = digitalRead(RightFlipperInput);
  
  if (!curTopSwitch0 && curTopSwitch0 != TopSwitch0Last){
    switch_top_0_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curTopSwitch1 && curTopSwitch1 != TopSwitch1Last){
    switch_top_1_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (curMidSwitch0 && curMidSwitch0 != MidSwitch0Last){
    switch_mid_0_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curMidSwitch1 && curMidSwitch1 != MidSwitch1Last){
    switch_mid_1_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (curBotSwitch0 && curBotSwitch0 != BotSwitch0Last){
    switch_bot_0_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curBotSwitch1 && curBotSwitch1 != BotSwitch1Last){
    switch_bot_1_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (!curStartButton && curStartButton != StartButtonLast){
    start_button_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (curLeftFlipperInput && curLeftFlipperInput != LeftFlipperInputLast){
    left_flipper_input_pub.publish(&empty_msg);
    nh.spinOnce();
  }
  if (curRightFlipperInput && curRightFlipperInput != RightFlipperInputLast){
    right_flipper_input_pub.publish(&empty_msg);
    nh.spinOnce();
  }

  TopSwitch0Last = curTopSwitch0;
  TopSwitch1Last = curTopSwitch1;
  MidSwitch0Last = curMidSwitch0;
  MidSwitch1Last = curMidSwitch1;
  BotSwitch0Last = curBotSwitch0;
  BotSwitch1Last = curBotSwitch1;
  StartButtonLast = curStartButton;
  LeftFlipperInputLast = curLeftFlipperInput;
  RightFlipperInputLast = curRightFlipperInput;
}

void setup(){
  // Setutp Inputs annd outputs
  pinMode(TopLight0, OUTPUT);
  pinMode(TopLight1, OUTPUT);
  pinMode(MidLight0, OUTPUT);
  pinMode(MidLight1, OUTPUT);
  pinMode(BotLight0, OUTPUT);
  pinMode(BotLight1, OUTPUT);
  
  pinMode(LeftFlipperOutput, OUTPUT);
  pinMode(RightFlipperOutput, OUTPUT);

  digitalWrite(LeftFlipperOutput, LOW);
  digitalWrite(RightFlipperOutput, LOW);

  pinMode(TopSwitch0, INPUT_PULLUP);
  pinMode(TopSwitch1, INPUT_PULLUP);
  //pinMode(MidSwitch0, INPUT_PULLUP);
  pinMode(MidSwitch1, INPUT_PULLUP);
  //pinMode(BotSwitch0, INPUT_PULLUP);
  pinMode(BotSwitch1, INPUT_PULLUP);

  pinMode(StartButton, INPUT_PULLUP);

  pinMode(LeftFlipperInput, INPUT);
  pinMode(RightFlipperInput, INPUT);
  
  nh.initNode();
  
  nh.advertise(switch_top_0_pub);
  nh.advertise(switch_top_1_pub);
  nh.advertise(switch_mid_0_pub);
  nh.advertise(switch_mid_1_pub);
  nh.advertise(switch_bot_0_pub);
  nh.advertise(switch_bot_1_pub);

  nh.advertise(left_flipper_input_pub);
  nh.advertise(right_flipper_input_pub);

  nh.advertise(start_button_pub);
  
  nh.subscribe(flip_sub);
  nh.subscribe(light_on_sub);
  nh.subscribe(light_off_sub);

  empty_msg.data = true;
}

void loop(){
  checkSwitches();
  nh.spinOnce();
}
