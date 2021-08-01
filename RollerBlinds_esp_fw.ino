//softesrial: TX IO12, RX IO14

// communication with ESP8266 module (procedure from STM32 side):
        // - STM turns on ESP power
        // - STM sends status(error, battery) and clock refresh request to ESP (frame COMM1)
        // - ESP connects to wifi, gets opening/closing time and time/date if requested
        // - ESP sends data to STM (frame COMM2)
        // - STM shuts off power to ESP

        // STM implements timeouts on communication

        //all frames are arrays of bytes (fixed length)

        // COMM1 frame: [bStatus, bPosition, bBatteryPercent, bBatteryDelta, bOpenHr, bOpenMin, bCloseHr, bCloseMin, bRequestTimeRefresh, bSum] len=10
        // COMM2 frame: [bOpenHr, bOpenMin, bCloseHr, bCloseMin, bTimeNowHr, bTimeNowMin, bTimeNowSec, bDateNowDate, bDateNowMonth, bDateNowYear, bEnableAuto, bManualPosition, bSum] len=13

        //sum is cheksum byte. sum of all previous bytes + 1 (with normal uintt8_t overflow)
        
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <numeric>

#define COMSER_BAUD 9600
#define COMSER_RX_PIN 14
#define COMSER_TX_PIN 12

// MQTT settings
const char* ssid = "planinsek";
const char* password = "cestasnegdrevomoka";
const char* mqtt_server = "192.168.1.112";
const char* clientId = "roller_blinds_nejc";
const char* topicBase = "home/blinds/nejc/#";
const char* enableTopic = "home/blinds/nejc/enabled";
const char* upTimeTopic = "home/blinds/nejc/upTime";
const char* downTimeTopic = "home/blinds/nejc/downTime";
const char* timeTopic = "home/time";
const char* setPositionTopic = "home/blinds/nejc/setPos";

const char* batteryPctTopic = "home/blinds/nejc/battery_pct";


SoftwareSerial comSer; //communication serial for comms with stm32 MCU

WiFiClient wifi_client;
PubSubClient client(wifi_client);

struct timeStruct {
  unsigned int hours;
  unsigned int minutes;
  unsigned int seconds;
};

bool gotTime = false;
bool gotEnableAuto = false;
bool gotUpTime = false;
bool gotDownTime = false;
bool gotManualPos = false;

timeStruct timeNow = {};
bool enableAuto = false;
timeStruct upTime = {};
timeStruct downTime = {};
long manualPos = 0;

bool comm1Received = false;
byte comm1Length = 10;
byte comm2Length = 13;

byte batteryPct = 0;

void setup() {
  Serial.begin(115200); //debug serial begin

  //void begin(uint32_t baud, SoftwareSerialConfig config, int8_t rxPin, int8_t txPin, bool invert, int bufCapacity = 64, int isrBufCapacity = 0);
  comSer.begin(COMSER_BAUD, SWSERIAL_8N1, COMSER_RX_PIN, COMSER_TX_PIN, false, 95, 11);
  comSer.setTimeout(500);
  pinMode(LED_BUILTIN, OUTPUT);

//  wifi_client.setTimeout(1000);
//  client.setSocketTimeout(2);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Connecting to MQQT");
  while(!client.connected()) {
    Serial.print(".");
    client.connect(clientId);
    delay(500);
  }

  Serial.println("");
  Serial.println("MQTT connected");
  client.subscribe(topicBase, 0);
  client.subscribe(timeTopic, 0);
  
}

void loop() {
  client.loop();
  
  if (!comm1Received) {
    byte comm1[comm1Length] = {0};
    byte numReceived = comSer.readBytes(comm1, comm1Length);
    if (numReceived > 0) {
          for (int i=0;i<numReceived;i++) {
            Serial.print(comm1[i]);Serial.print("\t");
          }
          Serial.println();
      if (numReceived == comm1Length) {
        // COMM1 frame: [bStatus, bPosition, bBatteryPercent, bBatteryDelta, bOpenHr, bOpenMin, bCloseHr, bCloseMin, bRequestTimeRefresh, bSum] len=10
        byte checksum = std::accumulate(comm1, comm1+comm1Length-1, 0) + 1;
        if (checksum == comm1[comm1Length-1]) {
          batteryPct = comm1[2];
    
          comm1Received = true;
    
          String msg(batteryPct);
          client.publish(batteryPctTopic, msg.c_str());
        }
        else {
          Serial.println("COMM1 Checksum mismatch");
        }
      }
      else {
        Serial.println("COMM1 Length mismatch");
      }
    }
  }
  
  if (gotTime && gotEnableAuto && gotUpTime && gotDownTime && gotManualPos && comm1Received) {
    Serial.println("GOT ALL DATA");
    client.disconnect();

    // send COMM2
    byte buff[comm2Length];
    buildComm2(buff);
    Serial.println("COMM2:");
    for (int i=0;i<comm2Length;i++) {
      Serial.print(buff[i]);Serial.print("\t");
    }
    Serial.println();
    comSer.write(buff, comm2Length);
    Serial.println("DONE");
    while(1) { delay(1000); };
  }

}

void callback(char* topic, byte* payloadRaw, unsigned int length) {
  byte payload[length];
  memcpy(payload, payloadRaw, length);
 
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Topic control
  if(strncmp(topic, timeTopic, strlen(topic)) == 0) {
    Serial.println("TIME");
    timeNow = parseTime((char*)payload);
    gotTime = true;
    Serial.print(timeNow.hours); Serial.println(timeNow.minutes);
  }
  else if(strncmp(topic, enableTopic, strlen(topic)) == 0) {
    Serial.println("ENABLE");
    
    if(strncmp((char*)payload, "on", length) == 0) {
      Serial.println("ENABLE ON");
      enableAuto = true;
    }
    else if(strncmp((char*)payload, "off", length) == 0) {
      Serial.println("ENABLE OFF");
      enableAuto = false;
    }
    gotEnableAuto = true;
  }
  if (strncmp(topic, upTimeTopic, strlen(topic)) == 0) {
    Serial.println("UP TIME");
    upTime = parseTime((char*)payload);
    gotUpTime = true;
  }
  else if (strncmp(topic, downTimeTopic, strlen(topic)) == 0) {
    Serial.println("DOWN TIME");
    downTime = parseTime((char*)payload);
    gotDownTime = true;
  }
  else if (strncmp(topic, setPositionTopic, strlen(topic)) == 0) {
    Serial.println("SET POSITION");
    manualPos = strtol((char*)payload, nullptr, 10);
    gotManualPos = true;
    Serial.println(manualPos);
  }

}

timeStruct parseTime(char* payload) {
  // assumes fromat HH:MM:SS
  unsigned int hours, minutes, seconds;
  char * endPtr;
  hours = strtol(payload, &endPtr, 10);
  minutes = strtol(endPtr+1, &endPtr, 10);
  seconds = strtol(endPtr+1, &endPtr, 10);

  timeStruct t{hours, minutes, seconds};
  return t;
}

void buildComm2(byte* buff) {
  // COMM2 frame: [bOpenHr, bOpenMin, bCloseHr, bCloseMin, bTimeNowHr, bTimeNowMin, bTimeNowSec, bDateNowDate, bDateNowMonth, bDateNowYear, bEnableAuto, bManualPosition, bSum] len=13
  buff[0] = upTime.hours;
  buff[1] = upTime.minutes;
  buff[2] = downTime.hours;
  buff[3] = downTime.minutes;
  buff[4] = timeNow.hours;
  buff[5] = timeNow.minutes;
  buff[6] = timeNow.seconds;
  buff[7] = 0;
  buff[8] = 0;
  buff[9] = 0;
  buff[10] = (byte)enableAuto;
  buff[11] = manualPos;
  buff[12] = std::accumulate(buff, buff+12, 0) + 1;
  
}
