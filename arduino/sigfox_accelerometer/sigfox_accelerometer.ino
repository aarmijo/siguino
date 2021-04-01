//third-party libraries (install per normal Arduino instructions: https://www.arduino.cc/en/hacking/libraries
#include <LowPower.h> //from RocketScream: https://github.com/rocketscream/Low-Power
#include <SparkFunLIS3DH.h> // from SparkFun: https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library

#include "util.h"
#include "Sigfox.h"
#include "accel.h"

#define VCC_MIN 2000
#define VCC_MAX 4000

//set to false for debug and to avoid sending messages but still take sensor readings
bool SEND_SIGFOX_MESSAGES = false;
//set to false to avoid waiting for the reception of downlink messages
bool RECEIVE_SIGFOX_MESSAGES = false;

//declare LIS3DH accelerometer
LIS3DH myIMU(SPI_MODE, 10); // constructed with parameters for SPI and cs pin number

//"global" vars
int period_count = 0;
int num_readings = 0;
volatile byte shock_state = LOW;
volatile int count = 0;
volatile bool interrupt_listen_shock = true;
bool shock_powered_down = false;
unsigned int seq_num = 0;
enum POWER_MODE {OFF = 0, ON = 1};
int init_vcc = 0;
uint8_t xXmax = 0;
uint8_t yYmax = 0;
uint8_t zZmax = 0;
uint8_t thresholdXaxis = 255; //0...255
uint8_t thresholdYaxis = 255;
uint8_t thresholdZaxis = 255; //0...255
uint8_t thresholdRotation = 63; // 127/2 ~ 45 degrees
uint8_t shockPin = 0;
uint8_t shockEventLasthour = 0;
uint8_t rotation_occurred = 0;
uint8_t battery_level = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LIGHT_PWR_PIN, LOW);
  Serial.begin(9600);

  pinMode(INT_PIN_SHOCK, INPUT);
  shockPin = digitalRead(INT_PIN_SHOCK);

  status_t accel_stat = Accel::setup_accel(&myIMU);
  bool accel_ok = accel_stat == IMU_SUCCESS;

  //attach interrupts so chip wakes if mag pulse or device move occurs
  attachInterrupt(digitalPinToInterrupt(INT_PIN_SHOCK), interrupt_shock, RISING);
  delay(5000);//Let system settle

  //sigfox comms test for debug purposes
  bool sigfox_ok = SigFox::test_sigfox_chip();

  init_vcc = Util::readVcc();
  Util::debug_print("Vcc = ", init_vcc, true);
  Util::debug_print("BatteryLevel = ", getBatteryLevel(init_vcc), true);

  Util::debug_print(F("Set sigfox sleep mode..."));
  SigFox::set_sigfox_sleep(true);
  //need to reset "volatile" variables here to avoid odd results
  shock_state = LOW;
  count = 0;

  // POST (power on self test) blinks for correct chip operation
  Util::blink_led(1);
  if (sigfox_ok) {
    Util::blink_led(1);
  } else {
    Util::debug_print(F("Sigfox chip not responding correctly"));
    delay(500);
  }
  if (accel_ok) {
    Util::debug_print(F("Accel chip. Accel OK."));
    uint8_t dataRead;
    myIMU.readRegister(&dataRead, 0x31);//cleared by reading
    Util::debug_print("Reg31H=" + String(dataRead, BIN));
    Util::blink_led(1);

  } else {
    Util::debug_print(F("LIS3DH chip not responding correctly"));
    delay(500);
  }

}//end "setup()"

void interrupt_shock() {
  if (shockPin == 0)
  {
    shockPin = 1;
  }
}

String stringHEX(unsigned long data, unsigned char numChars) {
  unsigned long mask  = 0x0000000F;
  mask = mask << 4 * (numChars - 1);
  String hex;

  for (unsigned int i = numChars; i > 0; --i) {
    hex += String(((data & mask) >> (i - 1) * 4), HEX);
    mask = mask >> 4;
  }

  return hex;
}

uint8_t getBatteryLevel(uint16_t ba)
{
  if (ba < VCC_MIN)
  {
    init_vcc = 0;
  }
  else if (VCC_MAX <= ba)
  {
    init_vcc = 15;
  }
  else
  {
    ba = (ba - VCC_MIN) / ((VCC_MAX - VCC_MIN) / 16);
  }
  return (ba);
}

String getSigFoxMessage(uint8_t sequence, uint8_t rotation_occurred, uint8_t accel_x, uint8_t accel_y, uint8_t accel_z, uint8_t battery_level,
                        uint8_t thresholdXaxis, uint8_t thresholdYaxis, uint8_t thresholdZaxis, uint8_t shockEventLasthour, uint8_t thresholdRotation, uint8_t messageType) {
  // Sigfox message of maximum 12 bytes
  // message type = 0 - keep-alive 
  // message type = 1 - accel-event
  String hexString = stringHEX(sequence, 2); // sequence -- 1 byte (2 hex chars)
  hexString += stringHEX(rotation_occurred, 2); // rotation_occurred -- 1 byte (2 hex chars)
  hexString += stringHEX(accel_x, 2); // accel_x -- 1 byte (2 hex chars)
  hexString += stringHEX(accel_y, 2); // accel_y -- 1 byte (2 hex chars)
  hexString += stringHEX(accel_z, 2); // accel_z -- 1 byte (2 hex chars)
  hexString += stringHEX(battery_level, 2); // battery_level -- 1 byte (2 hex chars)
  hexString += stringHEX(thresholdXaxis, 2); // thresholdXaxis -- 1 byte (2 hex chars)
  hexString += stringHEX(thresholdYaxis, 2); // thresholdYaxis -- 1 byte (2 hex chars)
  hexString += stringHEX(thresholdZaxis, 2); // thresholdZaxis -- 1 byte (2 hex chars)
  hexString += stringHEX(shockEventLasthour, 2); // shockEventLasthour -- 1 byte (2 hex chars)
  hexString += stringHEX(thresholdRotation, 2); // thresholdRotation -- 1 byte (2 hex chars)
  hexString += stringHEX(messageType, 2); // messageType -- 1 byte (2 hex chars)
  return hexString;
}

//main program loop
void loop() {

  Util::debug_print("\nPeriod counts : SigFox wait periods - " + String(period_count) + " : " + String(SIGFOX_WAIT_PERIODS));

  Util::debug_print("Setting Arduino Low Power Mode...");
  if (DEBUG_MODE) {
    delay(100);
  }

  // go to lowest power for maximum period (8 seconds)
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

  if (shockPin == 1)
  {
    shockEventLasthour++;
    Util::debug_print("\nShock events last SigFox wait period: " + String(shockEventLasthour));
    Util::debug_print(F("Wake up after INT. Accel:"));
    uint8_t dataReadX;
    uint8_t dataReadY;
    uint8_t dataReadZ;

    // accel x
    dataReadX = abs(Util::round_float(myIMU.readFloatAccelX() * 1000 / 7.8125));
    Util::debug_print("Current X accel abs (0-255): " + String(dataReadX));
    if (xXmax < dataReadX)
    {
      xXmax = dataReadX;
    }
    Util::debug_print("X accel max (0-255): " + String(xXmax));

    // accel y
    dataReadY = abs(Util::round_float(myIMU.readFloatAccelY() * 1000 / 7.8125));
    Util::debug_print("Current Y accel abs (0-255): " + String(dataReadY));
    if (yYmax < dataReadY)
    {
      yYmax = dataReadY;
    }
    Util::debug_print("Y accel max (0-255): " + String(yYmax));

    // accel z
    dataReadZ = abs(Util::round_float(myIMU.readFloatAccelZ() * 1000 / 7.8125));
    Util::debug_print("Current Z accel abs (0-255): " + String(dataReadZ));
    if (zZmax < dataReadZ)
    {
      zZmax = dataReadZ;
    }
    Util::debug_print("Z accel max (0-255): " + String(zZmax));

    shockPin = 0;
    num_readings++;

    if (dataReadX > thresholdXaxis || dataReadY > thresholdYaxis || dataReadZ > thresholdZaxis || dataReadY < thresholdRotation) {
      Util::debug_print(F("Set sigfox wake up to send event message..."));
      SigFox::set_sigfox_sleep(false);

      // rotation if accel in Y is lower than thresholdRotation
      if (dataReadY < thresholdRotation) {
        Util::debug_print("Rotation occurred around X axis!");
        rotation_occurred = 1;
      }

      if (dataReadX > thresholdXaxis || dataReadY > thresholdYaxis || dataReadZ > thresholdZaxis) {
        Util::debug_print("Above threshold event occurred!");
      }

      // get message of maximum 12 bytes
      String hexString = getSigFoxMessage(seq_num, rotation_occurred, xXmax, yYmax, zZmax, battery_level, thresholdXaxis, thresholdYaxis, thresholdZaxis, shockEventLasthour, thresholdRotation, 1);

      String msg_header = "Sigfox message (HEX): ";
      Util::debug_print(msg_header + hexString);

      if (SEND_SIGFOX_MESSAGES) {
        Util::debug_print(F("Sending over SigFox event message..."));

        digitalWrite(LED_PIN, HIGH); // turn the LED on (HIGH is the voltage level)

        String chip_response = SigFox::send_at_command("AT$SF=" + hexString, 6000);
        Util::debug_print("Reponse from sigfox module: " + chip_response);

        digitalWrite(LED_PIN, LOW); // turn the LED off by making the voltage LOW

      } else {
        Util::debug_print(F("Skipping Sigfox event message sending..."));
      }

      Util::debug_print(F("Set sigfox sleep mode..."));
      SigFox::set_sigfox_sleep(true);
      xXmax = 0;
      yYmax = 0;
      zZmax = 0;
      rotation_occurred = 0;
      shockEventLasthour = 0;
      num_readings = 0;
      period_count = 0;
    }
  }

  if (period_count >= SIGFOX_WAIT_PERIODS) {
    //only send data over sigfox every SIGFOX_WAIT_PERIODS (e.g. 8 = 8x8 seconds, ~ every minute)
    Util::debug_print(F("Sending period has occurred..."));
    Util::debug_print(F("Set sigfox wake up to send keep alive message..."));
    SigFox::set_sigfox_sleep(false);

    // sequence
    Util::debug_print("Message Sigfox - Sequence: " + String(seq_num));

    if (num_readings == 0) {
      xXmax = abs(Util::round_float(myIMU.readFloatAccelX() * 1000 / 7.8125));
      yYmax = abs(Util::round_float(myIMU.readFloatAccelY() * 1000 / 7.8125));
      xXmax = abs(Util::round_float(myIMU.readFloatAccelZ() * 1000 / 7.8125));
    }

    // x max
    Util::debug_print("Message Sigfox - Acc X max: " + String(xXmax));

    // y max
    Util::debug_print("Message Sigfox - Acc Y max: " + String(yYmax));

    // z max
    Util::debug_print("Message Sigfox - Acc Z max: " + String(zZmax));

    // battery level
    battery_level = getBatteryLevel(Util::readVcc());

    // get message of maximum 12 bytes
    String hexString = getSigFoxMessage(seq_num, rotation_occurred, xXmax, yYmax, zZmax, battery_level, thresholdXaxis, thresholdYaxis, thresholdZaxis, shockEventLasthour, thresholdRotation, 0);

    String msg_header = "Sigfox message (HEX): ";
    Util::debug_print(msg_header + hexString);

    if (SEND_SIGFOX_MESSAGES) {
      Util::debug_print(F("Sending over SigFox keep alive message..."));

      digitalWrite(LED_PIN, HIGH); // turn the LED on (HIGH is the voltage level)

      String chip_response = SigFox::send_at_command("AT$SF=" + hexString + ",1", 6000);
      Util::debug_print("Reponse from sigfox module: " + chip_response);

      if (RECEIVE_SIGFOX_MESSAGES) {
        Util::debug_print(F("Waiting for sigfox downlink response..."));
        delay(45000);
        String downlink_message = SigFox::recv_from_sigfox();
        Util::debug_print("Received message: " + downlink_message);
        Util::debug_print("Downlink: " + downlink_message.substring(7, 18));
        char string_hex[9];
        downlink_message.substring(7, 18).toCharArray(string_hex, 12);
        uint8_t downlinkThresholdXaxis;
        uint8_t downlinkThresholdYaxis;
        uint8_t downlinkThresholdZaxis;
        uint8_t downlinkThresholdRotation;
        sscanf(string_hex, "%x %x %x %x", &downlinkThresholdXaxis, &downlinkThresholdYaxis, &downlinkThresholdZaxis, &downlinkThresholdRotation);
        Util::debug_print("Downlink threshold X axis: " + String(downlinkThresholdXaxis));
        Util::debug_print("Downlink threshold Y axis: " + String(downlinkThresholdXaxis));
        Util::debug_print("Downlink threshold Z axis: " + String(downlinkThresholdXaxis));
        Util::debug_print("Downlink threshold rotation: " + String(downlinkThresholdRotation));
        thresholdXaxis = downlinkThresholdXaxis;
        thresholdYaxis = downlinkThresholdYaxis;
        thresholdZaxis = downlinkThresholdZaxis;
        thresholdRotation = downlinkThresholdRotation;
      }

      digitalWrite(LED_PIN, LOW); // turn the LED off by making the voltage LOW

    } else {
      Util::debug_print(F("Skipping Sigfox keep alive message sending..."));
    }

    Util::debug_print(F("Set sigfox sleep mode..."));
    SigFox::set_sigfox_sleep(true);
    xXmax = 0;
    yYmax = 0;
    zZmax = 0;
    rotation_occurred = 0;
    shockEventLasthour = 0;
    num_readings = 0;
    period_count = 0;
    seq_num++;
    seq_num = seq_num % 256;
  }
  period_count++;
  //delay(1000);
}// end loop()
