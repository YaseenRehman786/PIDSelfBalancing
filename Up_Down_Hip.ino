#include <Arduino.h>
#include "ODriveCAN.h"

// Documentation: https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html

/* Configuration of example sketch -------------------------------------------*/

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 1000000

// ODrive node_ids for two ODrives
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 3

// Uncomment the line that corresponds to your hardware setup.
#define IS_TEENSY_BUILTIN    // Teensy boards with built-in CAN interface (e.g. Teensy 4.1)
// #define IS_ARDUINO_BUILTIN   // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
// #define IS_MCP2515           // Boards with an external MCP2515 CAN controller

/* Board-specific includes ---------------------------------------------------*/

#if defined(IS_TEENSY_BUILTIN) + defined(IS_ARDUINO_BUILTIN) + defined(IS_MCP2515) != 1
#warning "Select exactly one hardware option at the top of this file."

#if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
#define IS_ARDUINO_BUILTIN
#warning "guessing that this uses HardwareCAN"
#else
#error "cannot guess hardware version"
#endif

#endif

#ifdef IS_ARDUINO_BUILTIN
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif // IS_ARDUINO_BUILTIN

#ifdef IS_MCP2515
#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"
#endif // IS_MCP2515

#ifdef IS_TEENSY_BUILTIN
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // Prevent Teensy compile error
#endif // IS_TEENSY_BUILTIN

/* Board-specific settings ---------------------------------------------------*/

/* Teensy */
#ifdef IS_TEENSY_BUILTIN
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
#endif // IS_TEENSY_BUILTIN

/* MCP2515-based extension modules */
#ifdef IS_MCP2515
MCP2515Class& can_intf = CAN;
#define MCP2515_CS 10      // Chip select pin for MCP2515
#define MCP2515_INT 2      // Interrupt pin for MCP2515 (check your board!)
#define MCP2515_CLK_HZ 8000000  // Frequency of the crystal oscillator

static inline void receiveCallback(int packet_size) {
  if (packet_size > 8) {
    return; // not supported
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

bool setupCan() {
  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE)) {
    return false;
  }
  CAN.onReceive(receiveCallback);
  return true;
}
#endif // IS_MCP2515

/* Arduinos with built-in CAN */
#ifdef IS_ARDUINO_BUILTIN
HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}
#endif // IS_ARDUINO_BUILTIN

/* Example sketch ------------------------------------------------------------*/

// Instantiate two ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1}; // Include all ODrive instances

// Structure to hold per-ODrive user data
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Create user data for each ODrive
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;

// Callback for heartbeat messages
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Callback for encoder feedback messages
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every incoming CAN message
void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

void setup() {
  Serial.begin(115200);
  // Wait for up to 3 seconds for Serial connection
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  Serial.println("Starting two-ODriveCAN demo");

  // Register callbacks for each ODrive
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);

  // Configure and initialize the CAN bus interface
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // halt execution
  }

  Serial.println("Waiting for both ODrives...");
  // Wait until both ODrives have sent their heartbeat
  while (!(odrv0_user_data.received_heartbeat && odrv1_user_data.received_heartbeat)) {
    pumpEvents(can_intf);
    delay(100);
  }

  Serial.println("Found both ODrives");

  // Example: Request bus voltage/current from ODrive0 (assuming shared power supply)
  Serial.println("Requesting bus voltage and current from ODrive0");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1000)) {
    Serial.println("vbus request failed!");
    while (true);
  }
  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  Serial.println("Enabling closed loop control on both ODrives...");
  // Loop until both ODrives enter closed loop control
  while ((odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) ||
         (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)) {
    odrv0.clearErrors();
    odrv1.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    // Allow time for state transition and CAN message propagation
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }

  Serial.println("Both ODrives running!");
}

// void loop() {
//   pumpEvents(can_intf); // Process incoming CAN messages

//   const float SINE_PERIOD = 2.0f; // Sine wave period in seconds
//   float t = 0.001 * millis();
//   float phase = t * (TWO_PI / SINE_PERIOD);

//   // Send position commands (sine wave) to both ODrives
//   float position = sin(phase);
//   float velocity_feedforward = cos(phase) * (TWO_PI / SINE_PERIOD);
//   odrv0.setPosition(position, velocity_feedforward);
//   odrv1.setPosition(position, velocity_feedforward);


//   // Print feedback from ODrive0 if available
//   if (odrv0_user_data.received_feedback) {
//     Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
//     odrv0_user_data.received_feedback = false;
//     Serial.print("odrv0-pos:");
//     Serial.print(feedback.Pos_Estimate);
//     Serial.print(",odrv0-vel:");
//     Serial.println(feedback.Vel_Estimate);
//   }
  
//   // Print feedback from ODrive1 if available
//   if (odrv1_user_data.received_feedback) {
//     Get_Encoder_Estimates_msg_t feedback = odrv1_user_data.last_feedback;
//     odrv1_user_data.received_feedback = false;
//     Serial.print("odrv1-pos:");
//     Serial.print(feedback.Pos_Estimate);
//     Serial.print(",odrv1-vel:");
//     Serial.println(feedback.Vel_Estimate);
//   }
// }

void loop() {
  pumpEvents(can_intf); // Process incoming CAN messages

  // Check if user has entered a new command via the Serial Monitor
  if (Serial.available() > 0) {
    // Read the input until newline
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove any whitespace
    if (input.length() > 0) {
      // Convert the input to a float
      float position = input.toFloat();
      float position2 = position *-1;
      Serial.print("Position 2 is ");
      Serial.println(position2);

      Serial.print("Setting ODrive positions to: ");
      Serial.println(position);

      // Send the position command to both ODrives
      // Here, velocity feedforward is set to 0.
      odrv0.setPosition(position, 0.0f);
      odrv1.setPosition(position2, 0.0f);
    }
  }
}
