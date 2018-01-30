

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
        void update(String state_name, uint8_t comms_status, double x_velocity, double theta, double battery_voltage);

      private:
        void _display_value(String value_name, double value, uint16_t value_color = HX8357_GREEN, uint8_t text_size = 2);
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

void TFTController::_display_value(String value_name, String value, uint16_t value_color = HX8357_GREEN, uint8_t text_size=2)
{
    tft.setTextSize(text_size);
    tft.setTextColor(HX8357_WHITE);
    tft.print(value_name + ': ');
    tft.setTextColor(HX8357_GREEN);
    tft.println(value);
}

unsigned long TFTController::update(String state_name, int8_t comms_status, double loop_rate, double x_velocity_cmd, double theta_cmd, double battery_voltage)
{
    tft.fillScreen(HX8357_BLACK);
    unsigned long start = micros();
    tft.setCursor(0, 0);
    tft.setTextColor(HX8357_WHITE);
    tft.setTextSize(5);
    tft.setTextColor(HX8357_RED);
    tft.println("Alexbot");
    // tft.drawLine(0, 6, tft.width(), 6, HX8357_RED);

    _display_value("State Name", state_name);

    if (comms_status < 0)
    {
        _display_value("ROS Communication Status", "Good", HX8357_GREEN);
    }
    else if (comms_status == 0)
    {
        _display_value("ROS Communication Status", "Disabled", HX8357_YELLOW);
    }
    else
    {
        _display_value("ROS Communication Status", "Error!", HX8357_RED);
    }

    _display_value("Loop Rate", String(loop_rate, 2) + 'Hz');
    _display_value("X Velocity", String(x_velocity_cmd, 2) + 'ms');
    _display_value("Theta", String(x_velocity_cmd, 2) + 'deg/s');
    _display_value("Battery Voltage", String(x_velocity_cmd, 2) + 'V');

    return micros() - start;
}
