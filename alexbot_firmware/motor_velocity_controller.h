class MotorVelocityController
{
    public:
      MotorVelocityController(String my_name, SabertoothSimplified *motor_interface,
                              int motor_id, WheelEncoderLS7366 *encoder_interface, int motor_max_power,
                              double Kp, double Ki, double Kd);

      void SetTargetVelocity(double target_vel);

    private:
      String my_name_;
      SabertoothSimplified *motor_interface_;
      WheelEncoderLS7366 *encoder_interface_;
      int motor_id_;
      int feedback_pin_;
      double Kp_;
      double Kd_;
      double Ki_;
      int motor_max_power_;
};

MotorVelocityController::MotorVelocityController(String my_name, SabertoothSimplified *motor_interface,
                                                 int motor_id, WheelEncoderLS7366 *encoder_interface, int motor_max_power,
                                                 double Kp = 0.5, double Ki = 0.0, double Kd = 0.0)
{
    // init the motor controller here
    this->my_name_           = my_name;
    this->motor_id_          = motor_id;
    this->motor_interface_   = motor_interface;
    this->encoder_interface_ = encoder_interface;
    this->motor_max_power_   = motor_max_power;
    this->Kp_                = Kp;
    this->Ki_                = Ki;
    this->Kd_                = Kd;
}

void MotorVelocityController::SetTargetVelocity(double target_vel)
{
    // Implementation of a PID controller
    // TODO: add make P and D terms work properly

    double current_vel = encoder_interface_->get_update().velocity;
    Serial.print(", current_vel=");
    Serial.print(current_vel);

    double pTerm = current_vel - target_vel;
    double iTerm = 0.0;
    double dTerm = 0.0;
    double output = int(Kp_ * pTerm + Ki_ * iTerm + Kd_ * dTerm);

    if ( output < -1 * motor_max_power_ ) {
      output = -1 * motor_max_power_;
    } else if ( output > motor_max_power_ ) {
      output = motor_max_power_;
    }

    Serial.println("");
    Serial.print(my_name_);
    Serial.print(", motor ID: ");
    Serial.print(motor_id_);
    Serial.print(", output=");
    Serial.print(output);
    Serial.print(", target_vel=");
    Serial.print(target_vel);

    if (abs(output) > 10)
    {
        motor_interface_->motor(motor_id_, output);
    }
}