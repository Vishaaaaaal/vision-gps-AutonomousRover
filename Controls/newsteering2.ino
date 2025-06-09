#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

// **Steering (Linear Actuator)**
int LPWM_PIN = 11;
int RPWM_PIN = 12;

// **Throttle Motor (Stepper)**
#define THROTTLE_DIR_PIN 6
#define THROTTLE_PUL_PIN 9
#define STEPS 500
bool is_at_max_position = false;

// **Potentiometer**
int potPin = A0;
float potValue = 0;  
float voltage = 0.0;
float current_steering_angle = 0.0;

// **Voltage to Steering Angle Mapping**
const float V_min = 0.20;  // Voltage at max left (-22°)
const float V_center = 2.55; // Voltage at center (0°)
const float V_max = 4.80;   // Voltage at max right (22°)
const float theta_min = -22.0;
const float theta_max = 22.0;

// **Calculate Linear Interpolation Constants**
const float m = (theta_max - theta_min) / (V_max - V_min);
const float b = theta_min - (m * V_min);

// **ROS Publishers**
std_msgs::Float32 pot_msg;
std_msgs::Float32 angle_msg;
ros::Publisher pot_pub("potentiometer_data", &pot_msg);
ros::Publisher angle_pub("steering_angle", &angle_msg);

// **Function to Convert Potentiometer Voltage to Steering Angle**
float getSteeringAngle(float voltage) {
    float steering_angle = (m * voltage) + b;
    return steering_angle;
}

// **Move Steering Based on Input**
void moveLinearActuator(int direction) {
    potValue = analogRead(potPin);  
    voltage = (potValue / 1023.0) * 5.0;  
    current_steering_angle = getSteeringAngle(voltage);

    if (direction == 1) {
        analogWrite(LPWM_PIN, 150);  // Extend actuator (Turn right)
        analogWrite(RPWM_PIN, 0);
    } else if (direction == -1) {
        analogWrite(LPWM_PIN, 0);
        analogWrite(RPWM_PIN, 150);  // Retract actuator (Turn left)
    } else {
        analogWrite(LPWM_PIN, 0);
        analogWrite(RPWM_PIN, 0);  // Stop movement
    }
}

// **Throttle Control - Move Stepper Once & Hold**
void moveStepper(bool forward) {
    digitalWrite(THROTTLE_DIR_PIN, forward ? HIGH : LOW);
    for (int i = 0; i < STEPS; i++) {
        digitalWrite(THROTTLE_PUL_PIN, HIGH);
        delayMicroseconds(200);
        digitalWrite(THROTTLE_PUL_PIN, LOW);
        delayMicroseconds(200);
    }
}

// **ROS Callback Function for Commands**
void messageCb(const std_msgs::String& msg) {
    String command = msg.data;

    if (command.startsWith("left")) {
        moveLinearActuator(-1);
    } else if (command.startsWith("right")) {
        moveLinearActuator(1);
    } else if (command == "center") {  
        moveLinearActuator(0);
    } else if (command == "throttle" && !is_at_max_position) {
        moveStepper(true);
        is_at_max_position = true;
    } else if (command == "reverse_throttle" && is_at_max_position) {
        moveStepper(false);
        is_at_max_position = false;
    } else if (command == "stop") {
        moveLinearActuator(0);
    }
}

// **ROS Subscriber**
ros::Subscriber<std_msgs::String> sub("nano_control", &messageCb);

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pot_pub);
    nh.advertise(angle_pub);

    pinMode(LPWM_PIN, OUTPUT);
    pinMode(RPWM_PIN, OUTPUT);
    pinMode(THROTTLE_DIR_PIN, OUTPUT);
    pinMode(THROTTLE_PUL_PIN, OUTPUT);
    pinMode(potPin, INPUT);

    Serial.begin(57600);
}

void loop() {
    nh.spinOnce();

    // **Read Potentiometer Value**
    potValue = analogRead(potPin);  
    voltage = (potValue / 1023.0) * 5.0;  

    // **Convert Voltage to Steering Angle**
    current_steering_angle = getSteeringAngle(voltage);

    // **Publish Potentiometer Data to ROS**
    pot_msg.data = voltage;
    pot_pub.publish(&pot_msg);

    // **Publish Steering Angle to ROS**
    angle_msg.data = current_steering_angle;
    angle_pub.publish(&angle_msg);

    Serial.print("Voltage: "); Serial.print(voltage);
    Serial.print(" | Steering Angle: "); Serial.println(current_steering_angle);

    delay(50);
}
