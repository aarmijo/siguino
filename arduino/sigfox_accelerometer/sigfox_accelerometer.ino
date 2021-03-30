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
uint8_t xXmax=0;
uint8_t yYmax=0;
uint8_t zZmax=0;
uint8_t totalXaxis=0;
uint8_t totalYaxis=0;
uint8_t totalZaxis=0;
uint8_t thresholdXaxis=10; //0...256
uint8_t thresholdYaxis=10; //0...256
uint8_t thresholdZaxis=10; //0...256
uint8_t shockPin = 0;
uint8_t shockEventLasthour=0;
uint8_t rotation_occurred = 0;
uint8_t battery_level = getBatteryLevel(Util::readVcc());

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LIGHT_PWR_PIN, LOW);
  Serial.begin(9600);

  pinMode(INT_PIN_SHOCK, INPUT);
  shockPin=digitalRead(INT_PIN_SHOCK);  

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
    Serial.print("Reg31H=");
    myIMU.readRegister(&dataRead, 0x31);//cleared by reading
    Serial.print(dataRead, BIN);
    Serial.println(";");
    Util::blink_led(1);

  } else {
    Util::debug_print(F("LIS3DH chip not responding correctly"));
    delay(500);
  }

}//end "setup()"

void interrupt_shock() {
  if (shockPin==0)
  {
      shockPin=1;
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
  if (ba<VCC_MIN)
  {
    init_vcc=0;
  }
  else if (VCC_MAX<=ba)
  {
    init_vcc=15;
  }
  else
  {
    ba=(ba-VCC_MIN)/((VCC_MAX-VCC_MIN)/16);
  }
  return(ba);
}

String getSigFoxMessage(uint8_t sequence, uint8_t rotation_occurred, uint8_t accel_x, uint8_t accel_y, uint8_t accel_z, uint8_t battery_level, uint8_t thresholdXaxis, uint8_t thresholdYaxis, uint8_t thresholdZaxis) {
    // Sigfox message of maximum 12 bytes
    String hexString = stringHEX(sequence, 2); // sequence -- 1 byte (2 hex chars)
    hexString += stringHEX(rotation_occurred, 2); // rotation_occurred -- 1 byte (2 hex chars)
    hexString += stringHEX(accel_x, 2); // accel_x -- 1 byte (2 hex chars)
    hexString += stringHEX(accel_y, 2); // accel_y -- 1 byte (2 hex chars)
    hexString += stringHEX(accel_z, 2); // accel_z -- 1 byte (2 hex chars)
    hexString += stringHEX(battery_level, 2); // battery_level -- 1 byte (2 hex chars)
    hexString += stringHEX(thresholdXaxis, 2); // thresholdXaxis -- 1 byte (2 hex chars)
    hexString += stringHEX(thresholdYaxis, 2); // thresholdYaxis -- 1 byte (2 hex chars)
    hexString += stringHEX(thresholdZaxis, 2); // thresholdZaxis -- 1 byte (2 hex chars)
    return hexString;
}

//main program loop
void loop() {

  //unsigned int ui = 0; //remove this

  //Util::debug_print("Setting Arduino Low Power Mode...");
  if (DEBUG_MODE) {
    delay(100);
  }
 
  // go to lowest power for maximum period (8 seconds)
  //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

  if (shockPin==1)
  {
    shockEventLasthour++;
    Util::debug_print(F("Wake up after INT. Accel:"));
    uint8_t dataReadX;
    uint8_t dataReadY;
    uint8_t dataReadZ;
    
    Serial.print("Accel: ");

    // accel x
    dataReadX = abs(Util::round_float(myIMU.readFloatAccelX()*1000/7.8125));
    Util::debug_print("Current X accel abs (0-255): " + String(dataReadX));
    if (xXmax<dataReadX)
    {
      xXmax=dataReadX;
    }
    totalXaxis += dataReadX;

    // accel y
    dataReadY = abs(Util::round_float(myIMU.readFloatAccelY()*1000/7.8125));
    Util::debug_print("Current Y accel abs (0-255): " + String(dataReadY));
    if (yYmax<dataReadY)
    {
      yYmax=dataReadY;
    }
    totalYaxis += dataReadY;

    // accel z
    dataReadZ = abs(Util::round_float(myIMU.readFloatAccelZ()*1000/7.8125));
    Util::debug_print("Current Z accel abs (0-255): " + String(dataReadZ));
    if (zZmax<dataReadZ)
    {
      zZmax=dataReadZ;
    }
    totalZaxis += dataReadZ;

    shockPin=0;
    num_readings++;

        if (dataReadX > thresholdXaxis ||  dataReadY > thresholdYaxis || dataReadZ > thresholdZaxis) {
          // send sigfox message

             Util::debug_print(F("Set sigfox wake up..."));
             SigFox::set_sigfox_sleep(false);

             // get message of maximum 12 bytes
             String hexString = getSigFoxMessage(seq_num, rotation_occurred, xXmax, yYmax, zZmax, battery_level, thresholdXaxis, thresholdYaxis, thresholdZaxis);
        
             String msg_header = "Sigfox message (HEX): ";
             Util::debug_print(msg_header + hexString);
    
            if (SEND_SIGFOX_MESSAGES) {
              Util::debug_print(F("Sending over SigFox above threshold alert message..."));
        
              digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        
              String chip_response = SigFox::send_at_command("AT$SF=" + hexString, 6000);
              Util::debug_print("Reponse from sigfox module: " + chip_response);
        
              digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
        
            } else {
              Util::debug_print(F("Skipping Sigfox message sending..."));
            }
        
          Util::debug_print(F("Set sigfox sleep mode..."));
          SigFox::set_sigfox_sleep(true);      
          totalXaxis = 0;
          totalYaxis = 0;
          totalZaxis = 0;
          xXmax = 0;
          yYmax = 0;
          zZmax = 0;
          rotation_occurred = 0;
          num_readings = 0;
          period_count = 0;
    }
    // TODO check against thresholds and send sigfox message if exceeded. Reset num_readings and period count within control block
    //Util::debug_print(F("Sending sigfox message because threshold exceeded..."));
  }  

  if (period_count >= SIGFOX_WAIT_PERIODS) {
    //only send data over sigfox every SIGFOX_WAIT_PERIODS (e.g. 8 = 8x8 seconds, ~ every minute)
    Util::debug_print(F("Sending period has occurred..."));
    Util::debug_print(F("Set sigfox wake up..."));
    SigFox::set_sigfox_sleep(false);

    // check rotation
    uint16_t meanXaxis = totalXaxis/num_readings;
    uint16_t meanYaxis = totalYaxis/num_readings;
    uint16_t meanZaxis = totalZaxis/num_readings;

    if (meanXaxis > 0 && meanYaxis > 0 && meanZaxis > 0) {
      rotation_occurred = 1;
    }

    // sequence    
    Util::debug_print("Message Sigfox - Sequence: " + String(seq_num));

    // x max
    Util::debug_print("Message Sigfox - Acc X max: " + String(xXmax));

    // y max
    Util::debug_print("Message Sigfox - Acc Y max: " + String(yYmax));

    // z max
    Util::debug_print("Message Sigfox - Acc Z max: " + String(zZmax));

    // battery level
    battery_level = getBatteryLevel(Util::readVcc());

    // get message of maximum 12 bytes
    String hexString = getSigFoxMessage(seq_num, rotation_occurred, xXmax, yYmax, zZmax, battery_level, thresholdXaxis, thresholdYaxis, thresholdZaxis);
        
    String msg_header = "Sigfox message (HEX): ";
    Util::debug_print(msg_header + hexString);

    if (SEND_SIGFOX_MESSAGES) {
      Util::debug_print(F("Sending over SigFox keep alive message..."));

      digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)

      String chip_response = SigFox::send_at_command("AT$SF=" + hexString + ",1", 6000);
      Util::debug_print("Reponse from sigfox module: " + chip_response);

      if (RECEIVE_SIGFOX_MESSAGES) {
        Util::debug_print(F("Waiting for sigfox downlink response..."));
        delay(45000);
        String downlink_message = SigFox::recv_from_sigfox();
        Util::debug_print("Received message: " + downlink_message);
        Util::debug_print("Downlink: " + downlink_message.substring(7, 15));
        char string_hex[9];
        downlink_message.substring(7, 15).toCharArray(string_hex, 9);
        uint8_t valor_primero;
        uint8_t valor_segundo;
        uint8_t valor_tercero;
        sscanf(string_hex, "%x %x %x", &valor_primero, &valor_segundo, &valor_tercero);  
        Util::debug_print("Valor primero: " + String(valor_primero));
        Util::debug_print("Valor segundo: " + String(valor_segundo));
        Util::debug_print("Valor tercero: " + String(valor_tercero));
        // TODO set thresholds with the downlink values
      }

      digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW

    } else {
      Util::debug_print(F("Skipping Sigfox message sending..."));
    }

    Util::debug_print(F("Set sigfox sleep mode..."));
    SigFox::set_sigfox_sleep(true);    
    totalXaxis = 0;
    totalYaxis = 0;
    totalZaxis = 0;
    xXmax = 0;
    yYmax = 0;
    zZmax = 0;
    rotation_occurred = 0;
    num_readings = 0;
    period_count = 0;
    seq_num++;
    seq_num = seq_num % 256;
  }
  period_count++;
  //delay(1000);
}// end loop()
