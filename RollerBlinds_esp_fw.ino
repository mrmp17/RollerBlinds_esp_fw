//softesrial: TX IO12, RX IO14

// communication with ESP8266 module (procedure from STM32 side):
        // - STM turns on ESP power
        // - STM sends status(error, battery) and clock refresh request to ESP (frame COMM1)
        // - ESP connects to wifi, gets opening/closing time and time/date if requested
        // - ESP sends data to STM (frame COMM2)
        // - STM shuts off power to ESP

        // STM implements timeouts on communication

        //all frames are arrays of bytes (fixed length)

        // COMM1 frame: [bStatus, bBatteryPercent, bBatteryDelta, bOpenHr, bOpenMin, bCloseHr, bCloseMin, bRequestTimeRefresh, bSum] len=9
        // COMM2 frame: [bOpenHr, bOpenMin, bCloseHr, bCloseMin, bTimeNowHr, bTimeNowMin, bTimeNowSec, bDateNowDate, bDateNowMonth, bDateNowYear, bNewTimes, bSum] len=12

        //sum is cheksup byte. sum of all previous bytes + 1 (with normal uintt8_t overflow)
        // if time values not available or requested set bytes to 0xFF
        //if new  open/close times from homeAssistant, set bNewTimes


#include <SoftwareSerial.h>

#define COMSER_BAUD 115200
#define COMSER_RX_PIN 14
#define COMSER_TX_PIN 12

SoftwareSerial comSer; //communication serial for comms with stm32 MCU

void setup() {
  Serial.begin(115200); //debug serial begin

  //void begin(uint32_t baud, SoftwareSerialConfig config, int8_t rxPin, int8_t txPin, bool invert, int bufCapacity = 64, int isrBufCapacity = 0);
  comSer.begin(COMSER_BAUD, SWSERIAL_8N1, COMSER_RX_PIN, COMSER_TX_PIN, false, 95, 11);
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);

}
