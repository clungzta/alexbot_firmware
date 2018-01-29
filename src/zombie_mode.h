
#include <Pozyx.h>
#include <Pozyx_definitions.h>
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

#define POSITIONING_ALGORITHM POZYX_POS_ALG_UWB_ONLY // try POZYX_POS_ALG_TRACKING for fast moving objects.
#define POZYX_DIMENSIONS      POZYX_2D
#define POZYX_HEIGHT          500 // HEIGHT not used in POZYX_2D

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

void print_pozyx_coordinates(coordinates_t coor)
{
    Serial.print("POS ID 0x");
    Serial.print(network_id, HEX);
    Serial.print(", x(mm): ");
    Serial.print(coor.x);
    Serial.print(", y(mm): ");
    Serial.print(coor.y);
    Serial.print(", z(mm): ");
    Serial.println(coor.z);
}

// function to manually set the Pozyx anchor coordinates
void setAnchorsManual()
{
    for (int i = 0; i < num_anchors; i++)
    {
        device_coordinates_t anchor;
        anchor.network_id = anchors[i];
        anchor.flag = 0x1;
        anchor.pos.x = anchors_x[i];
        anchor.pos.y = anchors_y[i];
        anchor.pos.z = heights[i];
        Pozyx.addDevice(anchor, remote_id);
    }
    if (num_anchors > 4)
    {
        Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, remote_id);
    }
}

class ZombieController
{
    public:
      ZombieController(Adafruit_GPS* gps);

    private:
      Adafruit_GPS* _gps;
      void _init_pozyx();
      void _init_gps();
      //   double _radar_threshold_max;
      //   double _infrared_threhold_max;
};

ZombieController::ZombieController()
{
    // Call separate init function from constructor (fixes errors)
    this->init();
}

void ZombieController::init_pozyx()
{
    // init the Pozyx DW1000 ToF RADAR localisation here
    serial.println("Initialising POZYX")

    if (Pozyx.begin() == POZYX_FAILURE)
    {
        Serial.println(F("ERROR: Unable to connect to POZYX shield"));
        Serial.println(F("Reset required"));
        delay(100);
        abort();
    }

    // sets the anchor manually
    setAnchorsManual();
    // sets the positioning algorithm
    Pozyx.setPositionAlgorithm(POSITIONING_ALGORITHM, POZYX_DIMENSIONS, NULL);
}

coordinates_t ZombieController::get_poxyx_position()
{
    coordinates_t position;
    int status = Pozyx.doPositioning(&position, POZYX_DIMENSIONS, POZYX_HEIGHT, POSITIONING_ALGORITHM);

    if (status != POZYX_SUCCESS)
    {
        uint8_t error_code;
        Pozyx.getErrorCode(&error_code);
        Serial.print("Pozyx Positioning Error: ");
        Serial.println(error_code, HEX);
    }

    return position;
}

void ZombieController::init_gps()
{
    // init the GPS reciever here

}
