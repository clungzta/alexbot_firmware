

#include <SPI.h>

// Weighting Constant for velocity exponentially weighted moving average
#define EWMA_ALPHA 1.0

// TAU = 2*PI (defining this saves some floating point operations)
#define TAU 6.28318530718

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
};

class WheelEncoderLS7366
{
    public:
        WheelEncoderLS7366(uint8_t encoder_id, uint8_t chip_select_pin, double counts_per_rev, double wheel_radius);
        WheelEncoderFeedback get_update();
        void reset_encoder();

    private:
        long request_encoder_position_();
        uint8_t encoder_id_;
        uint8_t chip_select_pin_;
        double counts_per_rev_;
        double wheel_radius_;
        long prev_count_;
        long latest_count_;
        unsigned long prev_stamp_;
        unsigned long latest_stamp_;
        double filtered_vel_;
};

WheelEncoderLS7366::WheelEncoderLS7366(uint8_t encoder_id, uint8_t chip_select_pin, double counts_per_rev, double wheel_radius)
{
    // init the motor controller here
    this->encoder_id_      = encoder_id;
    this->chip_select_pin_ = chip_select_pin;
    this->counts_per_rev_  = counts_per_rev;
    this->wheel_radius_    = wheel_radius;
    this->prev_count_      = 0;
    this->latest_count_    = 0;
    this->prev_stamp_      = 0.0;
    this->latest_stamp_    = 0.0;
    this->filtered_vel_    = 0.0;
    reset_encoder();
}

/**
 * @brief Calculates the distance traveled by the encoder
 * 
 * @return double, distance traveled by the wheel in metres
 */
WheelEncoderFeedback WheelEncoderLS7366::get_update()
{
    WheelEncoderFeedback feedback;
    feedback.raw_count = request_encoder_position_();

    feedback.distance_travelled = (double(feedback.raw_count) / counts_per_rev_) * TAU * wheel_radius_;
    double distance_travelled_prev = (double(prev_count_) / counts_per_rev_) * TAU * wheel_radius_;

    double t_delta = double(latest_stamp_ - prev_stamp_);
    double x_delta = feedback.distance_travelled - distance_travelled_prev;

    feedback.velocity_is_valid = (t_delta <= VELOCITY_CALCULATION_TIMEOUT);
    filtered_vel_ = EWMA_ALPHA * (x_delta / t_delta) + (1.0 - EWMA_ALPHA) * filtered_vel_;
    feedback.velocity = filtered_vel_;

    Serial.print("Encoder ");
    Serial.print(encoder_id_); 
    Serial.print(" distance_travelled: ");
    Serial.print(feedback.distance_travelled);
    Serial.print("m, velocity: ");
    Serial.print(feedback.velocity);
    Serial.print(" vel calc valid: ");
    Serial.println(feedback.velocity_is_valid);

    return feedback;
}

/**
 * @brief Gets the current position of the encoder
 * 
 * @return long, raw encoder ticks
 */
long WheelEncoderLS7366::request_encoder_position_()
{
    digitalWrite(chip_select_pin_, HIGH);

    SPI.transfer(0x60); // Request count

    unsigned int count1_val = SPI.transfer(0x00); // Read highest order byte
    unsigned int count2_val = SPI.transfer(0x00);
    unsigned int count3_val = SPI.transfer(0x00);
    unsigned int count4_val = SPI.transfer(0x00); // Read lowest order byte

    digitalWrite(chip_select_pin_, LOW);

    prev_stamp_ = latest_stamp_;
    latest_stamp_ = millis();

    prev_count_ = latest_count_;
    latest_count_ = ((long)count1_val << 24) + ((long)count2_val << 16) + ((long)count3_val << 8) + (long)count4_val;
    return latest_count_;
}

void WheelEncoderLS7366::reset_encoder()
{
    digitalWrite(chip_select_pin_, HIGH);
    SPI.transfer(CLR | CNTR);
    digitalWrite(chip_select_pin_, LOW);
}