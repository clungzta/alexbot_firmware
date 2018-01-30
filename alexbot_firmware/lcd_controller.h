

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"

// ESP32
#define STMPE_CS 32
#define TFT_CS 15
#define TFT_DC 33
#define SD_CS 14

#define TFT_RST -1

// Use hardware SPI and the above for CS/DC
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);

class TFTController
{
    public:
        TFTController();
        void init();
        unsigned long update(String state_name, int8_t comms_status, bool failsafe_status, double loop_rate, double x_velocity_cmd, double theta_cmd, double battery_voltage);

      private:
        void display_value_(String value_name, String value, uint16_t value_color, uint8_t text_size);
};

TFTController::TFTController()
{
}

void TFTController::init()
{
    // init the tft LCD controller here
    Serial.println("HX8357D Test!");

    // read diagnostics (optional but can help debug problems)
    uint8_t x = tft.readcommand8(HX8357_RDPOWMODE);
    Serial.print("Display Power Mode: 0x");
    Serial.println(x, HEX);
    x = tft.readcommand8(HX8357_RDMADCTL);
    Serial.print("MADCTL Mode: 0x");
    Serial.println(x, HEX);
    x = tft.readcommand8(HX8357_RDCOLMOD);
    Serial.print("Pixel Format: 0x");
    Serial.println(x, HEX);
    x = tft.readcommand8(HX8357_RDDIM);
    Serial.print("Image Format: 0x");
    Serial.println(x, HEX);
    x = tft.readcommand8(HX8357_RDDSDR);
    Serial.print("Self Diagnostic: 0x");
    Serial.println(x, HEX);
}

void TFTController::display_value_(String value_name, String value, uint16_t value_color = HX8357_GREEN, uint8_t text_size=2)
{
    tft.setTextSize(text_size);
    tft.setTextColor(HX8357_WHITE);
    tft.print(value_name);
    tft.print(": ");
    tft.setTextColor(HX8357_GREEN);
    tft.println(value);
}

unsigned long TFTController::update(String state_name, int8_t comms_status, bool failsafe_status, double loop_rate, double x_velocity_cmd, double theta_cmd, double battery_voltage)
{
    tft.fillScreen(HX8357_BLACK);
    unsigned long start = micros();
    tft.setCursor(0, 0);
    tft.setTextSize(5);
    tft.setTextColor(HX8357_RED);
    tft.println("Alexbot");
    // tft.drawLine(0, 6, tft.width(), 6, HX8357_RED);

    display_value_("State Name", state_name);

    if (comms_status > 0)
    {
        display_value_("ROS Communication Status", "Good", HX8357_GREEN);
    }
    else if (comms_status == 0)
    {
        display_value_("ROS Communication Status", "Disabled", HX8357_YELLOW);
    }
    else
    {
        display_value_("ROS Communication Status", "Error!", HX8357_RED);
    }

    display_value_("Loop Rate", String(loop_rate, 2) + "Hz");
    display_value_("Failsafe Status", String(failsafe_status));
    display_value_("X Velocity", String(x_velocity_cmd, 2) + "ms");
    display_value_("Theta", String(x_velocity_cmd, 2) + "deg/s");
    display_value_("Battery Voltage", String(x_velocity_cmd, 2) + "V");

    return micros() - start;
}
