#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // Prevent compile error

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
#define EARTH_GRAVITY_MS2 9.80665  //m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
/*---MPU6050 Control/Status Variables---*/
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gg;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            World-frame gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector


// Configuration of example sketch
#define CAN_BAUDRATE 1000000
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

// Instantiate ODrive objects for two nodes
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

// Define a drive offset (in degrees)
float driveOffset = 0.0; // Positive for reverse tilt, negative for forward tilt (or vice versa depending on your setup)
// Then, your effective target angle becomes:



// PID Constants
float Kp = 4.95; // Proportional gain --4.8 zhigler
float Ki = 35; // Integral gain --32 zhigler
float Kd = 0.21; // Derivative gain --0.18 zhigler

// PID Variables
bool initialPitchset = false;
float targetAngle = 0.00; // Target angle for balancing (0 degrees)
float currentError = 0, previousError = 0;
float integral = 0, derivative = 0;
float controlOutput = 0;




// Setup before the loop
unsigned long lastTime = 0;
float deltaTime = 0;
unsigned long currentTime = 0;
// int count = 0;
// #define NUM_READINGS 100
// float sumPitch = 0;
float currentPitch = 0.0;
float currentYaw = 0.0;
float currentRoll = 0.0;
float smoothedPitch = 0.0;
float smoothedYaw = 0.0;
float smoothedRoll= 0.0;
float alpha = 0.8;  // Smoothing factor (0 to 1)
float pitch;
float MAX_CONTROL_OUTPUT=10;

void setup() {
    Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);
  Serial.println("Starting ODriveCAN demo");
  // Register the feedback and heartbeat callbacks for both ODrives
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }
  Serial.println("Waiting for ODrive...");
  // Wait for both ODrives to send heartbeat
  while (!odrv0_user_data.received_heartbeat || !odrv1_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }
  Serial.println("found ODrive");
  // Read bus voltage and current for both ODrives
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) {
    Serial.println("vbus request failed for odrv0!");
    while (true); // spin indefinitely
  }
  Serial.print("ODRV0 DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("ODRV0 DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  if (!odrv1.request(vbus, 1)) {
    Serial.println("vbus request failed for odrv1!");
    while (true); // spin indefinitely
  }
  Serial.print("ODRV1 DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("ODRV1 DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  // Enabling closed loop control for both ODrives
  Serial.println("Enabling closed loop control...");

  // Set both ODrives to closed-loop control
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL ||
         odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    odrv1.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events to ensure the state is applied
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("ODrives running!");
  delay(1000);

  //MPU6050 Code

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }
  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);
    MPUIntStatus = mpu.getIntStatus();
    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  }
  delay(1000);
}



void loop(){
  currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0; // Calculate time elapsed in seconds
  
  pumpEvents(can_intf);
  
  if (!DMPReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { 
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    mpu.dmpGetGyro(&gg, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    delay(50);
  }
    //pitch = ypr[1] * RAD_TO_DEG;

    smoothedPitch = alpha * ypr[1] + (1.0 - alpha) * smoothedPitch;
    Serial.print("SmoothedPitch:");
    Serial.println(smoothedPitch);

    smoothedYaw = alpha * ypr[0] + (1.0 - alpha) * smoothedYaw;
    Serial.print("smoothedYaw:");
    Serial.println(smoothedYaw);

    smoothedRoll = alpha * ypr[2] + (1.0 - alpha) * smoothedRoll;
    Serial.print("smoothedRoll:");
    Serial.println(smoothedRoll);



if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        if (input.startsWith("kp=")) {
            Kp = input.substring(3).toFloat();
            Serial.print("Kp updated to: ");
            Serial.println(Kp);
        }
        if (input.startsWith("ki=")) {
            Ki = input.substring(3).toFloat();
            Serial.print("Ki updated to: ");
            Serial.println(Ki);
        }
        if (input.startsWith("kd=")) {
            Kd = input.substring(3).toFloat();
            Serial.print("Kd updated to: ");
            Serial.println(Kd);
        }
        if (input.startsWith("angle=")){
          targetAngle = input.substring(12).toFloat();
          Serial.print("targetAngle updated to: ");
          Serial.println(targetAngle);
        }
        if (input.startsWith("drive=")) {
        // Convert command value (e.g., 1 for forward, -1 for reverse)
        int command = input.substring(6).toInt();
        // Set drive offset angle (tweak the multiplier to adjust sensitivity)
        driveOffset = command * 0.01; // 5 degrees tilt for moderate speed
        Serial.print("Drive Offset updated to: ");
        Serial.println(driveOffset);
    } 
    }

    float effectiveTargetAngle = targetAngle + driveOffset;

    currentPitch = smoothedPitch;
    currentError = effectiveTargetAngle  - currentPitch;
    integral += currentError * deltaTime; // Integrate error
    derivative = (currentError - previousError) / deltaTime; // Derive error
    
    controlOutput = (Kp * currentError) + (Ki * integral) + (Kd * derivative);

    if (controlOutput > MAX_CONTROL_OUTPUT) {
     integral = integral - (controlOutput - MAX_CONTROL_OUTPUT) * deltaTime;  // Prevent integral from accumulating
    } else if (controlOutput < -MAX_CONTROL_OUTPUT) {
     integral = integral - (controlOutput + MAX_CONTROL_OUTPUT) * deltaTime;  // Prevent integral from accumulating
    }
    
    Serial.print("Kp: ");
    Serial.println(Kp * currentError);
    
    Serial.print("Ki: ");
    Serial.println(Ki * integral);
    
    Serial.print("Kd: ");
    Serial.println(Kd * derivative);

    Serial.print("Control Output: ");
    Serial.println(controlOutput);
    Serial.print("Error: ");
    Serial.println(currentError);

    //float MAX_VELOCITY = 10.0; // Set to appropriate motor limit
    //controlOutput = constrain(controlOutput, -MAX_VELOCITY, MAX_VELOCITY);
    
    
    odrv0.setVelocity(-controlOutput, 60.0f);
    odrv1.setVelocity(controlOutput, 60.0f);

    //odrv0.setTorque(-controlOutput);
    //odrv1.setTorque(controlOutput);

    previousError = currentError;
    lastTime = currentTime;
    Serial.print("Previous Error:");
    Serial.println(previousError);
    Serial.print("Delta Time:");
    Serial.println(deltaTime);





Serial.println(Kp);
Serial.println(Ki);
Serial.println(Kd);
Serial.println(targetAngle);
velocityfeedback();
}



void velocityfeedback(){
    if (odrv0_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
    odrv0_user_data.received_feedback = false;
    // Serial.print("odrv0-pos:");
    // Serial.print(feedback.Pos_Estimate);
    // Serial.print(",");
    Serial.print("odrv0-vel:");
    Serial.println(feedback.Vel_Estimate);
  }

if (odrv1_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = odrv1_user_data.last_feedback;
    odrv1_user_data.received_feedback = false;
    // Serial.print("odrv1-pos:");
    // Serial.print(feedback.Pos_Estimate);
    // Serial.print(",");
    Serial.print("odrv1-vel:");
    Serial.println(feedback.Vel_Estimate);
  }
}




