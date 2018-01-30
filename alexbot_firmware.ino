#include <Wire.h>

// // FIXME!
// #define BUFFER_LENGTH	32
// #include <Pozyx.h>
// #include <Pozyx_definitions.h>
#include <Adafruit_GPS.h>
#include <RPLidar.h>
//#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

#include "lcd_controller.h"
#include "serial_command.h"
#include "alexbot.h"

// This Sketch is intended to support ESP32 only (curently the only Dual-Core ESP on the market)!
TaskHandle_t Task1, Task2;
SemaphoreHandle_t baton;

#define LCD_REFRESH_INTERVAL 1000 //ms
#define RPLIDAR_MOTOR_PIN 3

AlexbotController alexbot;
SerialCommand sc;
TFTController touchscreen;
RPLidar lidar;

double measured_loop_rate;

/**
 * @brief main_task runs on ESP32 Core 0
 * 
 * @param parameter, pointer is passed 
 */
void main_task_func(void *parameter)
{
  String taskMessage = "main_task running on core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);

  // This loop runs continuously on core 0
  while (true)
  {
    unsigned long loop_start = millis();
    Serial.println("main_task at beginning of loop");

    // Take the Sephamore "baton" (defined globally)
    xSemaphoreTake(baton, portMAX_DELAY);

    // If Serial mode is used, we read commands from the serial port
    sc.ReadData();

    if (sc.message_type != -1)
    {
      Serial.print("valid message:");
      Serial.print(sc.message_type);
      Serial.print(",");
      Serial.print(sc.message_data1);
      Serial.print(",");
      Serial.println(sc.message_data2);
      sc.reset();
    }

    // Set Velocity
    if (sc.message_type == 'V')
    {
      Serial.print("Processing Velocity command from serial port on core: ");
      Serial.println(xPortGetCoreID());
      alexbot.process_velocity_command(sc.message_data1, sc.message_data2);
    }

    // Let go of the baton (so the other core can do some stuff)
    xSemaphoreGive(baton);
    Serial.println("main_task Done");

    // Give the other core some time to do its thing... 
    delay(50);

    // Can adjust global variables from the loop

    measured_loop_rate = 1000.0 / double(millis() - loop_start);
  }
}

/**
 * @brief auxillary_task runs on ESP32 Core 1
 * 
 * @param parameter, pointer is passed 
 */
void auxillary_task_func(void *parameter)
{
  String taskMessage = "auxillary_task running on core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);

  // This loop runs continuously on core 0
  while (true)
  {
    unsigned long loop_start = millis();
    Serial.println("auxillary_task at beginning of loop");

    // Take the Sephamore "baton" (defined globally)
    xSemaphoreTake(baton, portMAX_DELAY);

    // Read from the LIDAR
    // if (IS_OK(lidar.waitPoint()))
    // {
    //   float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    //   float angle = lidar.getCurrentPoint().angle;       //anglue value in degree
    //   bool startBit = lidar.getCurrentPoint().startBit;  //whether this point is belong to a new scan
    //   byte quality = lidar.getCurrentPoint().quality;    //quality of the current measurement

    //   //perform data processing here...
    // }
    // else
    // {
    //   analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

    //   // try to detect RPLIDAR...
    //   rplidar_response_device_info_t info;
    //   if (IS_OK(lidar.getDeviceInfo(info, 100)))
    //   {
    //     // detected...
    //     lidar.startScan();

    //     // start motor rotating at max allowed speed
    //     analogWrite(RPLIDAR_MOTOR, 255);
    //     delay(1000);
    //   }
    // }

    // Do Some Stuff
    if (millis() % LCD_REFRESH_INTERVAL == 0)
    {
      Serial.println("auxillary_task: Updating LCD");
      String current_state_id_str = String(alexbot.get_current_state_ID());

      int8_t serial_comms_status = 1;
      touchscreen.update(current_state_id_str, serial_comms_status, alexbot.check_failsafes(), measured_loop_rate, -1.0, -1.0, -1.0);
    }

    // Let go of the baton (so the other core can do some stuff)
    xSemaphoreGive(baton);
    Serial.println("auxillary_task Done");

    // Give the other core some time to do its thing...
    delay(50);

    // Can adjust global variables from the loop

  }
}

void setup()
{
    Serial.begin(115200);

    
    Serial.println("Initialising!");

    // Mutex ("baton") to be passed between cores to syncronise
    baton = xSemaphoreCreateMutex();

    alexbot.init();
    alexbot.set_current_state_ID(HALT_STATE);

    touchscreen.init();

    // mainControlLoop handles higher priority functions, including the motor control loop
    xTaskCreatePinnedToCore(
        main_task_func, /* Task function. */
        "Control Loop", /* String with name of task. */
        5000,           /* Stack size in words. */
        NULL,           /* Parameter passed as input of the task */
        10,             /* Priority of the task. */
        &Task1,         /* Task handle. */
        0);             /* Core ID to execute on. */

    Serial.print("Setup: created Task2 with priority = ");
    Serial.println(uxTaskPriorityGet(Task1));

    delay(500); // needed to start-up task1

    // mainControlLoop handles lower priority functions, including reading from the LIDAR, updating the TFT LCD and parsing GPS
    xTaskCreatePinnedToCore(
        auxillary_task_func, /* Task function. */
        "Auxillary Loop",    /* String with name of task. */
        5000,                /* Stack size in words. */
        NULL,                /* Parameter passed as input of the task */
        50,                  /* Priority of the task. */
        &Task2,              /* Task handle. */
        1);                  /* Core ID to execute on. */

    Serial.print("Setup: created Task2 with priority = ");
    Serial.println(uxTaskPriorityGet(Task2));

    Serial.println(F("Boot time benchmark                Time (microseconds)"));
}

void loop()
{
  Serial.print("Main Loop: priority = ");
  Serial.println(uxTaskPriorityGet(NULL));
  delay(5);
}
