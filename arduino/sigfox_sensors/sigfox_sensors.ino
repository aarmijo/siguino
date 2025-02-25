//third-party libraries (install per normal Arduino instructions: https://www.arduino.cc/en/hacking/libraries
#include <LowPower.h> //from RocketScream: https://github.com/rocketscream/Low-Power
#include <SparkFunLIS3DH.h> // from SparkFun: https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library
#include <Adafruit_BME280.h>

#include "util.h"
#include "Sigfox.h"
#include "accel.h"
#include "magnet.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

const byte BME_ADDRESS = 0x76;

//set to false for debug and to avoid sending messages but still take sensor readings
bool SEND_SIGFOX_MESSAGES = true;
//set to false to avoid waiting for the reception of downlink messages
bool RECEIVE_SIGFOX_MESSAGES = true;

//declare LIS3DH accelerometer
LIS3DH myIMU(SPI_MODE, 10); // constructed with parameters for SPI and cs pin number

//"global" vars
int period_count = SIGFOX_WAIT_PERIODS; //start at max so initial message is sent on startup
int num_readings = 0;
volatile byte shock_state = LOW;
volatile int count = 0;
volatile bool interrupt_listen_shock = true;
bool shock_powered_down = false;
volatile byte mag_state = HIGH;
volatile bool interrupt_listen_mag = true;
volatile bool state_changed_mag = false;
bool mag_opened = false;
unsigned int seq_num = 0;
enum POWER_MODE {OFF = 0, ON = 1};
int init_vcc = 0;
int total_light = 0;
float total_temp = 0;
float total_pressure = 0;
float total_humidity = 0;
float total_altitude = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(LIGHT_PWR_PIN, OUTPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT); //defaults to INPUT anyway, not strictly necessary
  digitalWrite(LIGHT_PWR_PIN, LOW);
  Serial.begin(9600);

  pinMode(INT_PIN_SHOCK, INPUT_PULLUP);
  pinMode(INT_PIN_MAG, INPUT_PULLUP);
  pinMode(MAG_PWR_PIN, OUTPUT);
  set_power_mag(POWER_MODE::ON);

  status_t accel_stat = Accel::setup_accel(&myIMU);
  bool accel_ok = accel_stat == IMU_SUCCESS;

  //attach interrupts so chip wakes if mag pulse or device move occurs
  attachInterrupt(digitalPinToInterrupt(INT_PIN_MAG), interrupt_mag, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INT_PIN_SHOCK), interrupt_shock, CHANGE);
  delay(300);//Let system settle

  //blink led to show power on
  //Util::blink_led(2);

  //sigfox comms test for debug purposes
  bool sigfox_ok = SigFox::test_sigfox_chip();

  init_vcc = Util::readVcc();
  Util::debug_print("Vcc = ", init_vcc, true);

  Util::debug_print(F("Set sigfox sleep mode..."));
  SigFox::set_sigfox_sleep(true);
  //need to reset "volatile" variables here to avoid odd results
  shock_state = LOW;
  count = 0;
  mag_state = digitalRead(INT_PIN_MAG);
  state_changed_mag = false;
  interrupt_listen_mag = true;

  // setup BME280 chip on appropriate I2C address
  bool bme_status = bme.begin(BME_ADDRESS);
  if (!bme_status) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
  }

  //set up magnetometer
  bool mag_ok = Magnetometer::checkMagnetometer();

  // POST (power on self test) blinks for correct chip operation
  Util::blink_led(1);
  if (sigfox_ok) {
    Util::blink_led(1);
  } else {
    Util::debug_print(F("Sigfox chip not responding correctly"));
    delay(500);
  }
  if (accel_ok) {
    Util::blink_led(1);
  } else {
    Util::debug_print(F("LIS3DH chip not responding correctly"));
    delay(500);
  }
  if (mag_ok) {
    Util::blink_led(1);
  } else {
    Util::debug_print(F("Magnetometer not responding correctly"));
    delay(500);
  }
  if (bme_status) {
    Util::blink_led(1);
  } else {
    Util::debug_print(F("Bosch temp chip not responding correctly"));
    delay(500);
  }

}//end "setup()"

void interrupt_shock() {
  if (interrupt_listen_shock) {
    shock_state = !shock_state;
    interrupt_listen_shock = false;
  }
}

void interrupt_mag() {
  if (interrupt_listen_mag) {
    state_changed_mag = true;
    mag_opened = !mag_state;
    mag_state = !mag_state;
    count++;
    interrupt_listen_mag = false;
  }
}

bool check_shock_occurred() {
  bool retval = false;
  if (shock_state == HIGH) {
    retval = true;
  }
  shock_state = LOW;
  if (shock_powered_down) {
    //re-enable shock component
    Util::debug_print(F("Re-enabling shock sensor..."));
    myIMU.begin();
    delay(500);
    shock_powered_down = false;
  }
  interrupt_listen_shock = true;
  return retval;
}

bool check_mag_occurred() {
  bool retval = false;
  if (state_changed_mag) {
    retval = true;
    state_changed_mag = false;
  }
  if (!interrupt_listen_mag) {
    Util::debug_print(F("Re-enabling mag sensor..."));
    set_power_mag(POWER_MODE::ON);
    delay(10);
  }
  mag_state = digitalRead(INT_PIN_MAG);

  interrupt_listen_mag = true;
  return retval;
}

void set_power_mag(int mode) {
  if (mode == POWER_MODE::ON) {
    digitalWrite(MAG_PWR_PIN, HIGH);
  } else {
    digitalWrite(MAG_PWR_PIN, LOW);
  }
}

float get_temperature_reading() {
  float temperature = 0.0000;
  temperature = bme.readTemperature();
  return temperature;
}

int get_light_reading() {
  digitalWrite(LIGHT_PWR_PIN, HIGH);
  int light_value =  analogRead(LIGHT_SENSOR_PIN);
  digitalWrite(LIGHT_PWR_PIN, LOW);
  return light_value;
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

//main program loop
void loop() {

  float f = 0;
  unsigned int ui = 0;

  //check interrupt state and remove interrupt if already occurred (i.e. switch off component)
  if ((!interrupt_listen_shock) && (!shock_powered_down)) {
    //if we're not listening anymore it means a shock has occurred
    // this means we can disable (unpower) the device until the next sending period
    Util::debug_print("Powering down shock sensor...");
    myIMU.writeRegister(LIS3DH_CTRL_REG1, 0x00);
    shock_powered_down = true;
  }

  Util::debug_print("Setting Arduino Low Power Mode...");
  if (DEBUG_MODE) {
    delay(100);
  }
 
  // go to lowest power for maximum period (8 seconds)
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

  if (period_count % SENSOR_READ_PERIOD == 0) {
    //SENSOR_READ_PERIOD determines how often to read sensor data
    // get temp
    Util::debug_print("Getting sensor information...");
    f = get_temperature_reading();
    // Lectura de temperatura
    Util::debug_print("Temperatura (ºC): " + String(f));
    total_temp += f;

    // get light level
    int light_reading = get_light_reading();
    Util::debug_print("Luz: " + String(light_reading));
    total_light += light_reading;

    //get pressure
    f = bme.readPressure() / 100.0F;
    Util::debug_print("Presión (mbar): " + String(f));
    total_pressure += f;

    //get altitude
    f = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Util::debug_print("Altitud (m): " + String(f));
    total_altitude += f;

    //get humidity
    f = bme.readHumidity();
    Util::debug_print("Humedad (%): " + String(f));
    total_humidity += f;

    num_readings++;
  }

  if (period_count >= SIGFOX_WAIT_PERIODS) {
    //only send data over sigfox every SIGFOX_WAIT_PERIODS (e.g. 8 = 8x8 seconds, ~ every minute)
    Util::debug_print(F("Sending period has occurred..."));
    Util::debug_print(F("Set sigfox wake up..."));
    SigFox::set_sigfox_sleep(false);

    // secuencia
    uint8_t secuencia = seq_num;
    Util::debug_print("Mensaje Sigfox - Secuencia: " + String(secuencia));

    // temperatura_media * 100
    ui = Util::round_float((total_temp / num_readings) * 100);
    uint16_t temperatura_media = ui;
    Util::debug_print("Mensaje Sigfox - Temperatura media * 100: " + String(temperatura_media));

    // luz_media
    ui = total_light / num_readings;
    uint16_t luz_media = ui;
    Util::debug_print("Mensaje Sigfox - Nivel medio de luz: " + String(luz_media));

    //check if shock interrupt has occurred and re-attach interrupts as necessary
    uint8_t choque_ocurrido = check_shock_occurred();
    Util::debug_print("Mensaje Sigfox - Choque ocurrido: " + String(choque_ocurrido));
    uint8_t mag_ocurrido = check_mag_occurred();
    Util::debug_print("Mensaje Sigfox - Mag ocurrido: " + String(mag_ocurrido));

    // nivel_bateria
    ui = (Util::readVcc() / 100) - MIN_BATT_LVL;
    uint16_t nivel_bateria = ui;
    Util::debug_print("Mensaje Sigfox - Nivel medio de batería: " + String(nivel_bateria));

    // presion_media
    ui = Util::round_float(total_pressure / num_readings);
    uint16_t presion_media = ui;
    Util::debug_print("Mensaje Sigfox - Presión media: " + String(presion_media));

    // humedad_media * 100
    ui = Util::round_float((total_humidity / num_readings) * 100);
    uint16_t humedad_media = ui;
    Util::debug_print("Mensaje Sigfox - Humedad media * 100: " + String(humedad_media));

    // altitud_media
    ui = min(Util::round_float((total_altitude / num_readings) / 10) - MIN_ALTITUDE, MAX_ALTITUDE);
    uint8_t altitud_media = ui;
    Util::debug_print("Mensaje Sigfox - Altitud media: " + String(altitud_media));

    // Sigfox message of maximum 12 bytes
    String hexString = stringHEX(choque_ocurrido, 2); // choque_ocurrido -- 1 byte (2 hex chars)
    hexString += stringHEX(mag_ocurrido, 2); // mag_ocurrido -- 1 byte (2 hex chars)
    hexString += stringHEX(temperatura_media, 4); // temperatura_media -- 2 bytes (4 hex chars)
    hexString += stringHEX(luz_media, 4); // luz_media -- 2 bytes (4 hex chars)
    hexString += stringHEX(nivel_bateria, 4); // nivel_bateria -- 2 bytes (4 hex chars)
    hexString += stringHEX(presion_media, 4); // presion_media -- 2 bytes (4 hex chars)
    hexString += stringHEX(humedad_media, 4); // humedad_media -- 2 bytes (4 hex chars)
        
    String msg_header = "Sigfox message (HEX): ";
    Util::debug_print(msg_header + hexString);

    if (SEND_SIGFOX_MESSAGES) {
      Util::debug_print(F("Sending over SigFox..."));

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
      }

      digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW

    } else {
      Util::debug_print(F("Skipping Sigfox message sending..."));
    }

    Util::debug_print(F("Set sigfox sleep mode..."));
    SigFox::set_sigfox_sleep(true);
    total_temp = 0;
    total_light = 0;
    total_pressure = 0;
    total_humidity = 0;
    total_altitude = 0;
    num_readings = 0;

    period_count = 0;
    seq_num++;
    seq_num = seq_num % 256;
  }
  period_count++;
  //delay(1000);
}// end loop()
