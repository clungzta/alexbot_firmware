#include "teleop_controller.h"
#include "motor_velocity_controller.h"

/***************************** STATE DEFINITIONS **************************************/
// These are the names of the states that the car can be in:

// SLEEP_STATE: Microcontroller enters deep sleep to conserve power
// Main computer gets turned OFF
// In this state: the Wi-Fi is polled occasionally to check whether the robot should "wake up" and do something
#define SLEEP_STATE             0

// HALT_STATE: Microcontroller in normal state, robot motor contollers disabled
// Main computer can be either ON or OFF
#define HALT_STATE              1

// BLUETOOTH_COMMAND_STATE: For use with Bluetooth Joystick android app
// (https://play.google.com/store/apps/details?id=org.projectproto.btjoystick)
// Main computer can be either ON or OFF
// Assisted teleop can be either ON or OFF
#define BLUETOOTH_TELEOP_STATE  2

// ZOMBIE_STATE: "Zombie Mode" is intended for Homing the robot to its docking station for a critical battery recharge
// Main computer gets turned OFF to conserve ~35W of power.
// Therefore: all localisation and navigation is performed ONBOARD OF THE ESP32
// Assisted teleop is ON in this state
// See README.md for more info about ZOMBIE_MODE
#define ZOMBIE_STATE            3

// SERIAL_COMMAND_STATE: for use with ROS or other serial driver
// Receives and processes commands, including velocity command
// Can also adjust config parameters ON the Microcontroller
#define SERIAL_COMMAND_STATE    4

/*************************************************************************************/

/**************************** ARDUINO PIN DEFINITIONS ********************************/
#define FAILSAFE_PIN       10   // To emergency stop switch
#define FAILSAFE_LED_PIN   13   // OUTPUT TO LED ON THE ARDUINO BOARD

// Motor driver Pins (UART Serial)
#define MOTOR_CONTROLLER_TX // S1 on the sabertooth 2x25A goes to pin 2

/************************************************************************************/

/************************************** CONFIG *************************************/

// Maximum allowable power to the motors
#define DRIVE_MOTORS_MAX_POWER 60

/***********************************************************************************/

// If a command from the RC or AI has not been recieved within WATCHDOG_TIMEOUT ms, will be switched to HALT state.
#define WATCHDOG_TIMEOUT 250

class Alexbot
{
  public:
    Alexbot()
    {
        // Initialise pins
        pinMode(FAILSAFE_LED_PIN, OUTPUT);
        pinMode(FAILSAFE_PIN, INPUT);
    }

    void Init() {

      // Seperate function for initialising objects
      // In Arduino, these init calls do not work from the class constructor

      // Initialise 9600 baud communication with the Sabertooth Motor Controller
      SoftwareSerial ST25port(NOT_A_PIN, 2);  // RX on no pin (unused), TX on pin 2 (to S1).
      ST32port.begin(9600);

      sabertooth = new SabertoothSimplified(ST25port);

      // Initialise Motor Controllers
      left_motor = new MotorController(
          "Left motor", sabertooth, 0,
          BRAKE_ACTUATOR_POSITION_SENSOR_PIN,
          BRAKE_FULLY_ENGAGED_POSITION, BRAKE_NOT_ENGAGED_POSITION,
          BRAKE_MOTOR_MAX_POWER);

      right_motor = new MotorController(
          "Right motor", sabertooth, 1,
          GEAR_ACTUATOR_POSITION_SENSOR_PIN,
          DRIVE_GEAR_POSITION, PARK_GEAR_POSITION,
          GEAR_MOTOR_MAX_POWER);
    }

    void process_velocity_command(double cmd_x_velocity = 0.0, double cmd_theta = 0.0)
    {
        // This function gets called repeatedly

        last_command_timestamp = millis();
        Serial.println("Processing command");

        // Will be changed into the HALT state if it is not safe to drive.
        check_failsafes();

        // State Machine
        switch (current_state_id)
        {
            case HALT_STATE:
                // We are in HALT_STATE

                x_velocity = 0.0;
                theta = 0.0;

                break;

            case RC_TELEOP_STATE:
            {
                // We are in RC_TELEOP_STATE
                // Lets act according to the PWM input commands from the RC reciever

                Serial.print(", desired_left=");
                Serial.print(left_vel_desired);

                Serial.print(", desired_right=");
                Serial.print(right_vel_desired);

                // FIXME: Change from velocity control to position control
                // Send command to the brake motor controller
                left_motor->SetTargetVelocity(left_vel_desired);
                right_motor->SetTargetVelocity(right_vel_desired);

                Serial.println("");
                break;
            }
        }

    }

    bool set_current_state_ID(uint8_t new_state_id)
    {
        // This function gets called when a change state is requested
        // Returns true on a successful transition

        // Code blocks within this switch statement are only called on change transient
        switch (new_state_id)
        {
            case HALT_STATE:
                // Do nothing on transition into HALT
                break;
            case RC_TELEOP_STATE:
                // Do nothing on transition into RC_TELEOP
                break;
            case AI_READY_STATE:
                // Do nothing on transition into AI
                break;
        }

        Serial.print("Changing state to: ");
        Serial.println(new_state_id);

        current_state_id = new_state_id;
        return true;
    }

    int get_current_state_ID()
    {
        // Return the ID of the state that we are currently in
        return current_state_id;
    }

    float read_pwm_value(int pwm_pin)
    {
        // Read a value from a PWM input
        // Used on RC control for all commands, and failsafe
        // Used on AI control for failsafe ONLY
        unsigned long pwm_time = pulseIn(pwm_pin, HIGH);
        return (float)pwm_time;
    }

    bool check_failsafes()
    {
        // This function will check all failsafes
        // If it is not safe to drive: the car will be switched to HALT_STATE
        // Pin 13 will be ON when it is safe to drive, otherwise OFF.

        // The failsafes include: a watchdog timer (i.e. an automatic shutdown if a command hasn't been recieved within 250ms)
        // Also included is a hardware switch.

        Serial.println("Checking failsafes!");
        bool watchdog_valid = ((millis() - last_command_timestamp) < WATCHDOG_TIMEOUT);
        bool failsafe_switch_engaged = digitalRead(FAILSAFE_PIN);

        Serial.print("failsafe_switch_engaged=");
        Serial.print(failsafe_switch_engaged);

        Serial.print(", watchdog_valid=");
        Serial.println(watchdog_valid);

        bool safe_to_drive = (watchdog_valid && failsafe_switch_engaged);

        if (!safe_to_drive)
        {
            set_current_state_ID(HALT_STATE);
        }

        digitalWrite(FAILSAFE_LED_PIN, safe_to_drive);

        return safe_to_drive;
    }


private:
    int current_state_id;
    long last_command_timestamp;
    Servo throttle_servo;

    SabertoothSimplified* sabertooth;

    MotorController* left_motor;
    MotorController* right_motor;
};
