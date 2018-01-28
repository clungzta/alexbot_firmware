
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <IRremote.h>

/******************* CONFIG **********************/

#define GPS_DIST_THRESHOLD_MIN 10000
#define INFRARED_DIST_THRESHOLD_MAX 3000

// what's the name of the hardware serial port for the GPS?
#define GPSSerial Serial2

// what's the name of the hardware serial port for the DW-1000 Tag?
#define RADARSerial Serial1

// FIXME: Pins for the IR reciever interupts
#define IR_PIN_FRONT_LEFT   3
#define IR_PIN_FRONT_CENTRE 4
#define IR_PIN_FRONT_RIGHT  5
#define IR_PIN_REAR_CENTRE  6

// Get these from the manufacturer
// DW-1000 ToF Tags
// the network id of the anchors: change these to the network ids of your anchors.
// Centre of the docking station is the origin
#define RADAR_TAG_ID_LEFT  0x1156
#define RADAR_TAG_ID_RIGHT 0x256B
#define RADAR_TAG_ID_REAR  0x3325

#define RADAR_TAG_SEPARATION     2500 // mm 

#define POSITIONING_ALGORITHM POZYX_POS_ALG_UWB_ONLY
#define DIMENSION             POZYX_3D

/**************************************************/

// Other Defines
#define RADAR_DIST_THRESHOLD_MAX GPS_DIST_THRESHOLD_MIN
#define RADAR_DIST_THRESHOLD_MIN INFRARED_DIST_THRESHOLD_MAX

// Initialisation of IR recievers
IRrecv irrecv_fl(IR_PIN_FRONT_LEFT);
IRrecv irrecv_fc(IR_PIN_FRONT_CENTRE);
IRrecv irrecv_fr(IR_PIN_FRONT_RIGHT);
IRrecv irrecv_rc(IR_PIN_REAR_CENTRE);

uint16_t tags[num_tags] = {0x0001};
uint16_t anchors[num_anchors] = {RADAR_TAG_ID_LEFT, RADAR_TAG_ID_RIGHT, RADAR_TAG_ID_REAR}; // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[num_anchors] = {-TAG_SEPARATION, 0, TAG_SEPARATION};                      // anchor x-coorindates in mm
int32_t anchors_y[num_anchors] = {0, TAG_SEPARATION, 0};                                    // anchor y-coordinates in mm

class ZombieController
{
    public:
      ZombieController();

    // private:
    //   double _radar_threshold_max;
    //   double _infrared_threhold_max;
};

ZombieController::ZombieController()
{
    // init the zombie mode controller here
}

