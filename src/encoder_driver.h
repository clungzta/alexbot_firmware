

#include <SPI.h>

// Weighting Constant for velocity exponentially weighted moving average
#define EWMA_ALPHA 1.0

// TAU = 2*PI (defining this saves some floating point operations)
#define TAU = 6.28318530718

// Max duration between position readings for a the velocity calculation to be considered valid
#define VELOCITY_CALCULATION_TIMEOUT 200 //ms

// LS3766 Commands
#define CLR B00000000
#define RD B01000000
#define WR B10000000
#define LOAD B11000000
#define MDR0 B00001000
#define MDR1 B00010000
#define DTR B00011000
#define CNTR B00100000
#define OTR B00101000
#define STR B00110000

struct WheelEncoderFeedback
{
    long raw_count;
    double distance_travelled;
    double velocity;
    bool velocity_is_valid;
}

class WheelEncoderLS7366
{
    public:
        LS7366Encoder(uint8_t encoder_id, uint8_t chip_select_pin, double counts_per_rev, double wheel_radius);
        WheelEncoderFeedback get_update();
        void reset_encoder();

    private:
        long _request_encoder_position();
        uint8_t _encoder_id;
        uint8_t _chip_select_pin;
        double _counts_per_rev;
        double _wheel_diameter;
        long _prev_count;
        long _latest_count;
        unsigned long _prev_stamp;
        double _filtered_vel;
};

WheelEncoderLS7366::WheelEncoderLS7366(uint8_t encoder_id, uint8_t chip_select_pin, double counts_per_rev, double wheel_radius)
{
    // init the motor controller here
    this->_encoder_id      = encoder_id;
    this->_chip_select_pin = chip_select_pin;
    this->_counts_per_rev  = counts_per_rev;
    this->_wheel_radius    = wheel_radius;
    this->_prev_count      = 0;
    this->_latest_count    = 0;
    this->_filtered_vel    = 0.0;
    this->_prev_stamp      = 0.0;
    this->_latest_stamp    = 0.0;
}

/**
 * @brief Calculates the distance traveled by the encoder
 * 
 * @return double, distance traveled by the wheel in metres
 */
WheelEncoderFeedback WheelEncoderLS7366::get_update()
{
    feedback WheelEncoderFeedback;
    feedback.raw_count = _request_encoder_position()

    feedback.distance_travelled = (double(feedback.raw_count) / this->counts_per_rev) * TAU * this->_wheel_radius;
    double distance_travelled_prev = (double(this->_prev_count) / this->counts_per_rev) * TAU * this->_wheel_radius;

    double t_delta = double(this->_latest_stamp - this->_prev_stamp);
    double x_delta = feedback.distance_travelled - distance_travelled_prev;

    feedback.velocity_is_valid = (t_delta <= VELOCITY_CALCULATION_TIMEOUT);
    this->_filtered_vel = EWMA_ALPHA * (x_delta / t_delta) + (1.0 - EWMA_ALPHA) * this->_filtered_vel;
    feedback.velocity = this->_filtered_vel;

    Serial.print('Encoder ');
    Serial.print(_encoder_id); 
    Serial.print(' distance_travelled: ');
    Serial.print(feedback.distance_travelled);
    Serial.print('m, velocity: ');
    Serial.print(feedback.velocity);
    Serial.print(' vel calc valid: ');
    Serial.println(feedback.velocity_is_valid);

    return feedback;
}

/**
 * @brief Gets the current position of the encoder
 * 
 * @return long, raw encoder ticks
 */
long WheelEncoderLS7366::_get_current_position()
{
    digitalWrite(_chip_select_pin, HIGH);

    SPI.transfer(0x60);               // Request count
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte

    digitalWrite(_chip_select_pin, LOW);

    this->_prev_stamp = this->_latest_stamp;
    this->_latest_stamp = millis();

    this->_prev_count = this->_latest_count;
    this->_latest_count = ((long)count1Value << 24) + ((long)count2Value << 16) + ((long)count3Value << 8) + (long)count4Value;
    return this->_latest_count;
}

void reset_encoder(int i)
{
    digitalWrite(_chip_select_pin, HIGH);
    SPI.transfer(CLR | CNTR);
    digitalWrite(_chip_select_pin, LOW);
}