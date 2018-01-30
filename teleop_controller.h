
/******************* CONFIG **********************/

#define TELEOP_MAX_LIN_VEL 0.5 // metres/s
#define TELEOP_MAX_ANG_VEL 1.0 // rad/s

#define ROBOT_MAX_DECEL_RATE 1.0 // m/s^2

/*************************************************/

#define REGULAR_TELEOP_MODE       0
#define SEPF_ASSISTED_TELEOP_MODE 1

struct Velocity
{
    double linear;
    double angular;
};

class TeleopController
{
    public:
        TeleopController();
        void change_state(uint8_t state_id);
        void set_input_sensitivity(double lin_sensitivity, double ang_sensitivity);
        Velocity process_command(double lin_vel, double ang_vel);

      private:
        uint8_t _state_id;
        double _lin_vel_scaling_factor;
        double _ang_vel_scaling_factor;
};

TeleopController::TeleopController()
{
}

void TeleopController::set_input_sensitivity(double lin_sensitivity, double ang_sensitivity)
{
    /**
     * @brief different input devices (e.g. bluetooth joystick) may have different sensitivities
     * lets make the sensitivity adaptable for each TeleopController
     * 
     * Note: the sensitivity values MUST convert joystick values to velocites in metres per second!
     */

    _lin_vel_scaling_factor = lin_sensitivity;
    _ang_vel_scaling_factor = ang_sensitivity;
}

void TeleopController::change_state(uint8_t state_id)
{
    Serial.print("Changing serial state to: ");
    Serial.println(state_id);
    _state_id = state_id;
}

Velocity TeleopController::process_command(double lin_vel, double ang_vel)
{
    Velocity outvel;

    if (_state_id == REGULAR_TELEOP_MODE)
    {
        // Apply scaling and constraints
        outvel.linear = constrain(_lin_vel_scaling_factor * lin_vel, -TELEOP_MAX_LIN_VEL, TELEOP_MAX_LIN_VEL);
        outvel.angular = constrain(_ang_vel_scaling_factor * ang_vel, -TELEOP_MAX_ANG_VEL, TELEOP_MAX_ANG_VEL);
    }
    else if (_state_id == SEPF_ASSISTED_TELEOP_MODE)
    {
        Serial.println("SEPF Assisted Teleop Not Yet Implemented! Ignoring Command...");
    }

    return outvel;
}