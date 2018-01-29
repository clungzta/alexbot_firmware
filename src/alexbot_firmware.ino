#include <Streaming.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

#include "lcd_controller.h"
#include "serial_command.h"
#include "alexbot.h"

#define LCD_REFRESH_INTERVAL 1000 //ms

Alexbot alexbot;
SerialCommand sc;
TFTController tft;

double loop_rate;

void setup()
{
    Serial.begin(115200);
    Serial.println("Initialising!");

    alexbot.Init();
    alexbot.set_current_state_ID(AI_READY_STATE);

    tft.init();

    Serial.println(F("Boot time benchmark                Time (microseconds)"));
}

void loop()
{

  unsigned long loop_start = millis();

  // If AI mode is used, we read commands from the serial port
  sc.ReadData();

  if ( sc.message_type != -1 ) {
       Serial.print("valid message:");
       Serial.print(sc.message_type);
       Serial.print(",");
       Serial.print(sc.message_data1);
       Serial.print(",");
       Serial.println(sc.message_data2);
       sc.reset();
  }

  // Set Velocity
  if (sc.message_type == 'V') {
    alexbot.process_command(sc.message_data1, sc.message_data2);
  }

  if (millis() % LCD_REFRESH_INTERVAL == 0)
  {
    tft.update(String(alexbot.getcurrentStateID()), alexbot.check_failsafes(), -1.0, -1.0, -1.0, -1.0);
  }

  double loop_rate = 1000.0 / double(millis() - loop_start);
}
