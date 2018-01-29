# alexbot_firmware
V2.0 Firmware for Alexbot. https://hackaday.io/project/26127-alexbot  
Intended target for Dual-Core ESP32 low-cost embedded platform.

Currently Under Development!
  
---

Hardware Used:

- Adafruit HUZZAH32 - ESP32 (240 MHz dual core Tensilica LX6 microcontroller w/ Bluetooth + Wi-Fi) 
- Adafruit Ultimate GPS
- Adafruit TFT FeatherWing - 3.5 480x320 Touchscreen
- Sabertooth 2x25A Regenerative Motor Controller
- Triple LS7366R Quadrature Encoder Buffer
- BNO-055 IMU

---
 
## State Machine:
### 0. Halt:
The Robot will not move. ESP32 in low energy mode, irregularly polling for requests to turn the computer on.
### 1. ROS Serial Commands:
Bidirectional communication will occur between ROS and the ESP32.

This can control the motors and adjust state (e.g. flags) within the program.
### 2. Unassisted Teleop (e.g. from Bluetooth or Wi-Fi Control)
Velocity commands from a joystick get mapped to wheel velocities, then are sent directly to the motors.
### 3. Assisted Teleop (e.g. from Bluetooth or Wi-Fi Control)
Velocity commands from a joystick are used as Vin input to the SEPF Based Collision Avoidance algorithm proposed by (Qasim, 2016).

AssistedTeleopController() avoids obstacles by fusing user command input with repulsion vectors of objects detected in the LIDAR scan.
### 4. Zombie mode
Zombie Mode is intended for Homing the robot to the docking station for a critical recharge even when the main computer (i.e. ROS localisation and navigation stack) is off.

![alt text](https://cdn4.iconfinder.com/data/icons/miscellaneous-icons-3/200/monster_zombie_hand-256.png "Zombie")

Zombie mode gets enabled when either:
the batteries reach a low level (detected by coulomb counter), and the HOMING_INSTINCTS flag has been enabled by ROS.
The user has manually engaged it

Zombie mode has its own (sub-)state machine, as described in the table:

| Localisation Method               | Condition |
| --------------------              |:-------------:|
| GPS                               | if Distance to docking station > 10m |
| DW1000 RADAR                      | if (3m < Distance to docking station  < 10m)  AND (VLOS to docking station) |
| Infrared Beacon Navigation        | if Distance to docking station < 3m |
| Successfully Homed                | if appropriate voltage detected on docking station spring contacts |

#### GPS
Uses a GPS receiver fused with wheel encoders to perform waypoint navigation.
Appropriate velocity Commands are generated and sent to the AssistedTeleopController() for obstacle avoidance.

#### DW1000 RADAR
In this mode: ToF RADAR Beacon Trilateration is used to guide the robot towards the docking station.

Three DW1000 Beacons are to be placed “to the left of”, “to the right of” and “behind” the docking station respectively (from the docked robots frame of reference). See green dots on figure for beacon placement.

As the robot approaches the docking station: “2D Least Squares Trilateration” is used to calculate the 2D position of the robot.

#### Infrared Beacon Navigation
Appropriate velocity Commands are generated and sent to the AssistedTeleopController() for obstacle avoidance.

For localisation when the robot is within 3m visual LOS of the docking station. This allows to determine the angle of the docking station. Three IR (38KHz) LEDs are placed on the dock, namely “LEFT”, “MIDDLE” and “RIGHT”. IR receiver(s) are placed on the outer extremities of the robot (where appropriate, NOTE: More recievers results in greater FOV)

Synced by a short sync pulse (from all IR transmitters), followed by single pulses from the LEFT, MIDDLE, then RIGHT respectively. See timing diagram:
![alt text](https://photos.app.goo.gl/DKFhUayQKq2sYl8x1 "IR Timing Diagram")

Replication of XR-210 protocol as detailed by (Smith, 2010)
https://sites.google.com/site/mechatronicsguy/robot-vac-hack .
  
---  
### Acknowledgements

Firmware was originally forked and repurposed from "Linda", code for the self-driving car by [The Robotics Club](https://www.theroboticsclub.org/)

### References

Smith, G. (2010). Robot Vac Hack - The Mechatronics Guy. [online] Sites.google.com. Available at: https://sites.google.com/site/mechatronicsguy/robot-vac-hack [Accessed 27 Jan. 2018].

Qasim, M. ; Kim, K. (2016), 'Super-ellipsoidal Potential Function for Autonomous Collision Avoidance of a Teleoperated UAV', World Academy of Science, Engineering and Technology, International Science Index 109, International Journal of Mechanical, Aerospace, Industrial, Mechatronic and Manufacturing Engineering, 10(1), 164 - 169.

