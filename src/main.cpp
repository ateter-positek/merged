//Includes
#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <WiFi.h>
#include <VL6180X.h>
#include <ClosedCube_OPT3001.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_NeoPixel.h>
#include <MQTT_Topics.h>
extern "C"
{
#include <CAENRFIDLib_Light.h>
}

#define LED_PIN 27
#define NUM_LEDS 80
#define Ethernet_or_Wifi 1
#define WIFI_SSID "717"
#define WIFI_PASS "Brownian@e10k.com"
// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192, 168, 12, 2);
IPAddress myDns(8, 8, 8, 8);

// SPI pin assignments

#define PIN_CS 5
#define PIN_INT 15
#define PIN_RST 26
#define W5500_RESET_PIN 13
#define RFID_RESET_PIN 4


Client* client;
PubSubClient* mqttClient;


byte mac[6];

char usn[18];  // 48-bit MAC address as string (12 characters for hex digits, 5 for colons, and 1 for null terminator)
char* firmwareVersionNo;
char* mqtt_user ;
const char* mqtt_server = "192.168.12.110";
char* mqtt_password;
char* mqtt_client_id;
char device_id[18];
uint16_t mqtt_port = 1883;
int setup_finished = false;

//CAEN Reader definitions
CAENRFIDErrorCodes ec;
char model[MAX_MODEL_LENGTH];
char serial[MAX_SERIAL_LENGTH];
char source[MAX_LOGICAL_SOURCE_NAME];
const uint32_t rf_power = 200;
char ch;

//Light definitions
//Chase
int chasePixel = 0;
unsigned long lastChaseUpdate = 0;
const unsigned long chaseInterval = 25;

//Theater
int theaterChasePixel = 0;
int theaterChaseCycle = 0;
unsigned long lastTheaterChaseUpdate = 0;
const unsigned long theaterChaseInterval = 25;



int16_t _connect(void **port_handle, int16_t port_type, void *port_params);
int16_t _disconnect(void *port_handle);
int16_t _tx(void *port_handle, uint8_t *data, uint32_t len);
int16_t _rx(void *port_handle, uint8_t *data, uint32_t len, uint32_t ms_timeout);
int16_t _clear_rx_data(void *port_handle);
void _enable_irqs(void);
void _disable_irqs(void);

// other utility functions
uint64_t get_ms_timestamp(void);

static void onError(CAENRFIDErrorCodes ec);
static void printHex(uint8_t* vect, uint16_t length, char* result);

typedef struct
{
    uint8_t serialNum;
    uint32_t baudrate;
    uint8_t dataBits;
    uint8_t stopBits;
    uint8_t parity;
    uint8_t flowControl;
} RS232_params;

static CAENRFIDReader reader = {
    .connect = _connect,
    .disconnect = _disconnect,
    .tx = _tx,
    .rx = _rx,
    .clear_rx_data = _clear_rx_data,
    .enable_irqs = _enable_irqs,
    .disable_irqs = _disable_irqs
};

RS232_params port_params = {
    .serialNum = 2, // tobe updated wih the correct serial num //old code--> .com = "/dev/ttyACM0"
    .baudrate = 921600,  //921600
    .dataBits = 8,
    .stopBits = 1,
    .parity = 0,
    .flowControl = 0,
};

//VL6180X Sensor
struct VL6180XSENSOR {
  bool enabled;
  int sensorNum;
  int channel;
  int threshold;
  int sensorMode;
  int state;
};

struct VL6180XSENSOR vl6180xsensors[10];

//OPT3001 Sensor
struct OPT3001SENSOR {
  bool enabled;
  int sensorNum;
  int channel;
  float threshold;
  int sensorMode;
  int state; //0 for low, 1 for high
};

typedef enum {
  CONTINUOUS = 0,
  EDGE = 1
} SensorMode;

typedef enum {
  SensorReadError = -4,
  ReaderInitError = -3,
  VL6180XSensorInitError = -2,
  OPT3001SensorInitError = -1,
  Success = 0
} LogCode;								 

struct LogMessage {
  LogCode code;
  std::string message;
	   
};

struct OPT3001SENSOR opt3001sensors[10];

//LED
struct LedState {
 uint32_t color;
 uint32_t prevColor;
 unsigned long expirationTime;
};

LedState* ledStates;
int numLeds;
bool isPatternRunning = false;

enum LogLevel {
    LOG_POST = 0,
    LOG_ERROR = 1,
    LOG_WARNING = 2,
    LOG_STATUS = 3,
    LOG_INFORMATIONAL = 4,
    LOG_DEBUG = 5
};

// Set the default log level
LogLevel globalLogLevel = LOG_STATUS;

// Create a lookup table of string representations for the log levels
const char* LOG_LEVEL_STRINGS[] = {
    "POST",
    "Error",
    "Warning",
    "Status",
    "Informational",
    "Debug"
};		   
										 										   
//Objects
ClosedCube_OPT3001 opt3001;
VL6180X vl6180x;
QWIICMUX mux;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

//Function declarations

//Setup functions
void initUSN();
void initMux();
void initNetworking();
void initNetworkingSetup();
void initMQTT();
void initSetup();
void initReader();
void initSettings(char* message);
void initSensorStructs();
void initReaderSettings(char* message);
void initVL6180X_Sensor(char * message, int sensorNum);
void initOpt3001_Sensor(char* message, int sensorNum);
void connectToMQTT();

//MQTT Functions
void mqttLoop();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishLog(LogLevel log_level, const char* message, const char* errorcode);
void publishVL6180XRange(uint8_t range, int sensorNum);
void publishOPT3001Lux(float lux, int sensorNum);
void publishTagRead(char* epc);
void postComplete(char* message);

void reboot();

//Loop functions
void checkSensors();
void checkReader();

//Light controls
void lightControls(char* message);
void setSingleLED(int index, int red, int green, int blue, unsigned long timeout);
void setRangeLEDs(int start, int end, int red, int green, int blue, unsigned long timeout);
void chase(uint32_t color);
void theaterChase(uint32_t color);

//Other functions
uint8_t* hex_decode (const char *in, size_t len, uint8_t *out);

void setup() {


  Serial.begin(115200);
  Wire.begin();
  
Serial.println("Powering Up..");
  //Set W5500 reset and RFID reset
    pinMode(W5500_RESET_PIN, OUTPUT);
    pinMode(RFID_RESET_PIN, OUTPUT_OPEN_DRAIN);

  //set W5500 and RFID reset to low
  digitalWrite(RFID_RESET_PIN, LOW);
  digitalWrite(W5500_RESET_PIN, LOW);

  //wait 5000ms for caps to charge and stabalize
  delay(5000);

Serial.println("Booting now..");
  strcpy(source, "Source_0");


  numLeds = strip.numPixels();
  ledStates = new LedState[numLeds];
  for(int i=0; i<numLeds; i++) {
    ledStates[i].color = 0;
    ledStates[i].prevColor = 0;
    ledStates[i].expirationTime = 0;
  }


  Serial.print("Starting setup");
  strip.begin();

  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, 100, 0, 0); // set all pixels to red
  }
  strip.show(); // update the strip

  //Initialize USN
  initUSN();
    endpoint = "ENDPOINT/" + std::string(usn);
    // MQTT Topics - Inbound
    SETTINGS = endpoint + "/SETTINGS";
    READER_SETTINGS = endpoint + "/PUM/READER/SETTINGS";
    READER_START = endpoint + "/PUM/READER/START_INVENTORY";
    READER_STOP = endpoint + "/PUM/READER/STOP_INVENTORY";
    POSTCOMPLETE = endpoint + "/POSTCOMPLETE";
    REBOOT = endpoint + "/REBOOT";
    VL6180X_1_SETTING = endpoint + "/PUM/SENSOR/VL6180X/1/SETTINGS";
    VL6180X_2_SETTING = endpoint + "/PUM/SENSOR/VL6180X/2/SETTINGS";
    VL6180X_3_SETTING = endpoint + "/PUM/SENSOR/VL6180X/3/SETTINGS";
    VL6180X_4_SETTING = endpoint + "/PUM/SENSOR/VL6180X/4/SETTINGS";
    OPT3001_1_SETTING = endpoint + "/PUM/SENSOR/OPT3001/1/SETTINGS";
    OPT3001_2_SETTING = endpoint + "/PUM/SENSOR/OPT3001/2/SETTINGS";
    OPT3001_3_SETTING = endpoint + "/PUM/SENSOR/OPT3001/3/SETTINGS";
    OPT3001_4_SETTING = endpoint + "/PUM/SENSOR/OPT3001/4/SETTINGS";
    LIGHTS = endpoint + "/PUM/LIGHT/STRIP";

    // MQTT Topics - Outbound
    INIT = endpoint + "/INITIALIZE";
    LOGS = endpoint + "/LOGS";
    VL6180X_1_RANGE = endpoint + "/PUM/SENSOR/VL6180X/1/RANGE";
    VL6180X_2_RANGE = endpoint + "/PUM/SENSOR/VL6180X/2/RANGE";
    VL6180X_3_RANGE = endpoint + "/PUM/SENSOR/VL6180X/3/RANGE";
    VL6180X_4_RANGE = endpoint + "/PUM/SENSOR/VL6180X/4/RANGE";
    OPT3001_1_LUX = endpoint + "/PUM/SENSOR/OPT3001/1/LUX";
    OPT3001_2_LUX = endpoint + "/PUM/SENSOR/OPT3001/2/LUX";
    OPT3001_3_LUX = endpoint + "/PUM/SENSOR/OPT3001/3/LUX";
    OPT3001_4_LUX = endpoint + "/PUM/SENSOR/OPT3001/4/LUX";
    TAG_READ = endpoint + "/PUM/READER/TAG_READ";
    LAST_WILL = endpoint + "/STATUS";

  Serial.println("USN initialized");
  Serial.print("Endpoint: ");
  Serial.println(endpoint.c_str());
  Serial.println("Initializing networking");
   //set W5500 reset to high
  digitalWrite(W5500_RESET_PIN, HIGH);
  delay(500);
  initNetworkingSetup();
  Serial.println("Networking initialized");
  

  //Initialize MQTT
  initMQTT();
  Serial.println("MQTT initialized"); 
  //Powering up RFID reader
  Serial.println("Powering up RFID reader");

  //Initialize RFID reader
  initReader();
  Serial.println("Reader initialized");

  publishLog(LOG_POST, "Reader initialized", "E0001");

  //Initialize Multiplexer
  initMux();
  Serial.println("Mux initialized");

  publishLog(LOG_POST, "Mux initialized", "E0002");

  delay(1000);
  strip.clear();
  strip.show(); // update the strip

  initSetup();
  publishLog(LOG_POST, "Setup sequence complete", "E0003");
  Serial.println("Setup complete");
}

void loop() {
  unsigned long now = millis();
  for(int i=0; i<numLeds; i++) {
    if(ledStates[i].expirationTime > 0 && now >= ledStates[i].expirationTime) {
    // State has expired, return to previous color
    strip.setPixelColor(i, ledStates[i].prevColor);
    strip.show();
    // Reset the expiration time
    ledStates[i].expirationTime = 0;
    }
  }

  if(isPatternRunning) {
    //led_red, led_green, led_blue not defined in chat gpt code

    // chase(strip.Color(led_red, led_green, led_blue));
    // theaterChase(strip.Color(led_red, led_green, led_blue));
  }

  mqttLoop();
  checkSensors();
  checkReader();
 //Serial.println("Reader LOOP");
 //delay(100);
   //delay(1000);
  
}

void initUSN()
{
  // Initialize EEPROM
  EEPROM.begin(6);

  // Check the first byte of EEPROM to determine if the USN has been set
  uint8_t val = EEPROM.read(0);

  if (val != 0x10) {  // If the first byte is not DE, we assume USN has not been set
    Serial.println("USN not loaded, please input the last 2 bytes in format 'FF:FF'");

    char temp[6];  // Temporary buffer to read user input
    while (Serial.available() < 5);  // Wait until 5 characters are available (2 bytes in hex and 1 colon)

    // Read the user input
    for (int i = 0; i < 5; i++) {
      temp[i] = Serial.read();
    }
    temp[5] = '\0';  // Null-terminate the string

    // Parse the user input
    uint8_t result[2];
    sscanf(temp, "%hhx:%hhx", &result[0], &result[1]);

    Serial.print(result[0], HEX);
    Serial.print(":");
    Serial.println(result[1], HEX);


    // Write the USN to EEPROM
    EEPROM.write(0, 0x10);
    EEPROM.write(1, 0xAA);
    EEPROM.write(2, 0x00);
    EEPROM.write(3, 0x11);
    EEPROM.write(4, result[0]);
    EEPROM.write(5, result[1]);

    EEPROM.commit();

    Serial.println("Initial USN set, restarting..");
   // publishLog("POST", "Initial USN set");
    ESP.restart();
  } else {
    // USN has been set, read it from EEPROM
    uint8_t usn_bytes[6];
    for (int i = 0; i < 6; i++) {
      usn_bytes[i] = EEPROM.read(i);
    }

    // Format USN as a string
    sprintf(usn, "%02X:%02X:%02X:%02X:%02X:%02X", usn_bytes[0], usn_bytes[1], usn_bytes[2], usn_bytes[3], usn_bytes[4], usn_bytes[5]);\

    String usnString(usn);
    String usnLoaded = "USN loaded: " + usnString;
    Serial.print(usnLoaded);
    //publishLog("POST", usnLoaded.c_str());
    Serial.println(usn);
  }
}

uint8_t* hex_decode (const char *in, size_t len, uint8_t *out)
{
  unsigned int i, t, hn, ln;

  for (t = 0,i = 0; i < len; i+=2,++t) {
    hn = in[i] > '9' ? in[i] - 'A' + 10 : in[i] - '0';
    ln = in[i+1] > '9' ? in[i+1] - 'A' + 10 : in[i+1] - '0';
    out[t] = (hn << 4 ) | ln;
  }

  return out;
}

void initMux() {
  Wire.setClock(400000); // Set I2C bus speed to 100kHz
  uint32_t busSpeed = Wire.getClock(); // Get the I2C bus speed
  Serial.print("I2C Speed: ");
  Serial.print(busSpeed);
  Serial.println(" Hz");

  if (mux.begin() == false)
  {
    Serial.println("Mux not detected. Freezing...");
    publishLog(LOG_POST, "Multiplexer not detected", "E0004");
    while (1)
      ;
  }


  Serial.println("Mux detected");
  publishLog(LOG_POST, "Multiplexer detected; setting port to 0", "E0005");


  mux.setPort(0); //Connect master to port labeled '0' on the mux

  byte currentPortNumber = mux.getPort();
  Serial.print("CurrentPort: ");
  Serial.println(currentPortNumber);
}

void initNetworkingSetup() {
  if (Ethernet_or_Wifi == 1) {
    // Ethernet

    EEPROM.begin(6);  // Initialize EEPROM
    for (int i = 0; i < 6; ++i) {
      mac[i] = EEPROM.read(i); // Read MAC address from EEPROM
    }
    EEPROM.end();
    Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    //pass mac to device_id
    sprintf(device_id, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Set RESET and INTERRUPT pins
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, HIGH); // Deassert the reset
    pinMode(PIN_INT, INPUT); // Set the interrupt pin as an input
    
    Serial.println("Initializing Ethernet...");
    Ethernet.init(PIN_CS); 
    delay(2000); // Wait for the W5500 to initialize
  // start the Ethernet connection:
  // Serial.println("Initialize Ethernet");
  //   Ethernet.begin(mac, ip, myDns);


  //    Serial.print("  Static assigned IP ");
  //    Serial.println(Ethernet.localIP());


  Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      while (true) {
        delay(1000);
        Serial.println("Not detected");
        delay(1); // do nothing, no point running without Ethernet hardware
      }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip, myDns);
    
  } else {
    Serial.print("  DHCP assigned IP ");
    Serial.println(Ethernet.localIP());
  }
  // give the Ethernet shield a second to initialize:
  delay(1000);
  client = new EthernetClient();
  } 
  else {
    // Wi-Fi
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }

    // Connected to Wi-Fi
    Serial.println("Connected to WiFi");
    Serial.print("Local IP address: ");
    Serial.println(WiFi.localIP());
    client = new WiFiClient();
  }

  Serial.println("Created new client object");
  mqttClient = new PubSubClient(*client);
  Serial.println("Created new mqttClient object");
  delay(1000);
}

void initNetworking() {

  if (Ethernet_or_Wifi == 1) {
    // Ethernet
    // Ethernet.begin(mac, ip, myDns);


    //  Serial.print("  Static assigned IP ");
    //  Serial.println(Ethernet.localIP());

  //start the Ethernet connection:
  Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      while (true) {
        delay(1000);
        Serial.println("Not detected");
        delay(1); // do nothing, no point running without Ethernet hardware
      }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip, myDns);

  } else {
    Serial.print("  DHCP assigned IP ");
    Serial.println(Ethernet.localIP());
  }
  // give the Ethernet shield a second to initialize:
  delay(1000);
  client = new EthernetClient();
  }
  else {
    // Wi-Fi
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }

    // Connected to Wi-Fi
    Serial.println("Connected to WiFi");
    Serial.print("Local IP address: ");
    Serial.println(WiFi.localIP());
    client = new WiFiClient();
  }
  Serial.println("Created new client object");
  mqttClient = new PubSubClient(*client);
  Serial.println("Created new mqttClient object");
}


void initMQTT() {
  mqttClient->setServer(mqtt_server, mqtt_port);
  mqttClient->setCallback(mqttCallback);

  Serial.println("Connecting to MQTT");
  publishLog(LOG_POST, "Connecting to MQTT", "E0006" );
  connectToMQTT();
}

void initSetup() {
  Serial.println("Publishing Initialize to Node RED");
  publishLog(LOG_POST, "Publishing Initilizaing command to Node RED", "E0007" );

  char payload[10];
  snprintf(payload, sizeof(payload), "%d", true);
  mqttClient->publish(INIT.c_str(), payload);

  initSensorStructs();

  //loop until we receive the PostComplete topic
  while(setup_finished == false) {
    mqttClient->loop();
    unsigned long now = millis();
    for(int i=0; i<numLeds; i++) {
      if(ledStates[i].expirationTime > 0 && now >= ledStates[i].expirationTime) {
      // State has expired, return to previous color
      strip.setPixelColor(i, ledStates[i].prevColor);
      strip.show();
      // Reset the expiration time
      ledStates[i].expirationTime = 0;
      }
    }
  }

  strip.clear();
  strip.show();
}

void initSensorStructs() {
//initialize defaults for the structs
  for (int i = 0; i <= 4; i++) {
    struct VL6180XSENSOR vlsensor = {
      .enabled = false,
      .sensorNum = 0,
      .channel = 0,
      .threshold = 0,
      .sensorMode = 0,
      .state = 0
    };

    vl6180xsensors[i] = vlsensor;

    struct OPT3001SENSOR optsensor = {
      .enabled = false,
      .sensorNum = 0,
      .channel = 0,
      .threshold = 0,
      .sensorMode = 0,
      .state = 0
    };

    opt3001sensors[i] = optsensor;
  }

}

void connectToMQTT() {
  while (!mqttClient->connected()) {
    //Check for network connection before trying to connect to MQTT
    if ((Ethernet_or_Wifi == 1 && Ethernet.linkStatus() != 1) || (Ethernet_or_Wifi == 0 && WiFi.status() != WL_CONNECTED)) {
      Serial.println(Ethernet.linkStatus());
      Serial.println("Network connection lost. Re-establishing connection...");
      // Delete old client and mqttClient objects
      delete mqttClient;
      Serial.println("Deleted old mqttClient object");
      delete client;
      Serial.println("Deleted old client object");
      initNetworking(); // Attempt to re-establish network connection
    }

    if (mqttClient->connect(device_id, LAST_WILL.c_str(), 1, true, "offline")) {
      Serial.println("Connected to MQTT");
													   
      publishLog(LOG_POST, "Successfully connected to MQTT", "E0007");
      //Subscribe to topics
      mqttClient->subscribe(SETTINGS.c_str());
      mqttClient->subscribe(READER_SETTINGS.c_str());
      mqttClient->subscribe(VL6180X_1_SETTING.c_str());
      mqttClient->subscribe(VL6180X_2_SETTING.c_str());
      mqttClient->subscribe(VL6180X_3_SETTING.c_str());
      mqttClient->subscribe(VL6180X_4_SETTING.c_str());
      mqttClient->subscribe(OPT3001_1_SETTING.c_str());
      mqttClient->subscribe(OPT3001_2_SETTING.c_str());
      mqttClient->subscribe(OPT3001_3_SETTING.c_str());
      mqttClient->subscribe(OPT3001_4_SETTING.c_str());
      mqttClient->subscribe(POSTCOMPLETE.c_str());
      mqttClient->subscribe(REBOOT.c_str());
      mqttClient->subscribe(READER_SETTINGS.c_str());
      mqttClient->subscribe(READER_START.c_str());
      mqttClient->subscribe(READER_STOP.c_str());
      mqttClient->subscribe(LIGHTS.c_str());

    } else {
      delay(5000);
    }
  }
}
void initReader() {

uint32_t get_rf_power; 
  digitalWrite(RFID_RESET_PIN, LOW);
  delay(100);
  digitalWrite(RFID_RESET_PIN, HIGH);
  delay(500);

  ec = CAENRFID_Connect(&reader, CAENRFID_RS232, &port_params);
  Serial.println(ec);
  onError(ec);

  //read reader model and serial number
  ec = CAENRFID_GetReaderInfo(&reader, model, serial);
  onError(ec);
  printf("Reader info : %d, %s, %s\n", ec, model, serial);

  //configure RF power level
  ec = CAENRFID_SetPower(&reader, rf_power);
  onError(ec);
  ec = CAENRFID_GetPower(&reader, &get_rf_power);
  onError(ec);
  printf("Set power to %d mW : %d\n", get_rf_power, ec);

}

void mqttLoop() {
  if (!mqttClient->connected()) {
    connectToMQTT();
  }
  mqttClient->loop();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {

  String topicString(topic);
  String incomingTopic = "Incoming MQTT message: " + topicString;
  Serial.println(topicString);
 // publishLog("INFO", incomingTopic.c_str());
  //Convert the payload to a null-terminated string
  char message[length+1];
  for (unsigned int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';

  //Process the received message based on the topic
  if (strcmp(topic, SETTINGS.c_str()) == 0) {
    initSettings(message);
  } else if (strcmp(topic, READER_SETTINGS.c_str()) == 0) {
    initReaderSettings(message);
  } else if (strcmp(topic, VL6180X_1_SETTING.c_str()) == 0) {
    initVL6180X_Sensor(message, 1);
  } else if (strcmp(topic, VL6180X_2_SETTING.c_str()) == 0) {
    initVL6180X_Sensor(message, 2);
  } else if (strcmp(topic, VL6180X_3_SETTING.c_str()) == 0) {
    initVL6180X_Sensor(message, 3);
  } else if (strcmp(topic, VL6180X_4_SETTING.c_str()) == 0) {
    initVL6180X_Sensor(message, 4);
  } else if (strcmp(topic, OPT3001_1_SETTING.c_str()) == 0) {
    initOpt3001_Sensor(message, 1);
  } else if (strcmp(topic, OPT3001_2_SETTING.c_str()) == 0) {
    initOpt3001_Sensor(message, 2);
  } else if (strcmp(topic, OPT3001_3_SETTING.c_str()) == 0) {
    initOpt3001_Sensor(message, 3);
  } else if (strcmp(topic, OPT3001_4_SETTING.c_str()) == 0) {
    initOpt3001_Sensor(message, 4);
  } else if (strcmp(topic, POSTCOMPLETE.c_str()) == 0) {
    postComplete(message);
  } else if (strcmp(topic, REBOOT.c_str()) == 0) {
    reboot();
  } else if (strcmp(topic, READER_START.c_str()) == 0) {

  } else if (strcmp(topic, READER_STOP.c_str()) == 0) {

  } else if (strcmp(topic, LIGHTS.c_str()) == 0) {
    lightControls(message);
  }
}

void checkSensors() {

  Serial.println(" ");
  Serial.println(" ");
  Serial.println("################################################ ");
    Serial.println(" ");
  Serial.println(" ");
for (int i = 1; i < 5; i++) {
    //Serial.print("Sensor ");
    VL6180XSENSOR vlsensor = vl6180xsensors[i];
    if (vlsensor.enabled) { //
      mux.setPort(vlsensor.channel);
//delay(1);

     // uint16_t read = vl6180x.readRangeContinuousMillimeters();
         uint16_t read = vl6180x.readRangeSingleMillimeters();
     if (vl6180x.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
      //Serial.println(read);
           if (vlsensor.state)
  {
      Serial.printf("Sensor VL  %d: %d.000000 on Port: %d         #####TRIGGERED#####\n", i, read, vlsensor.channel);

  }else{
      Serial.printf("Sensor VL  %d: %d.000000 on Port: %d  \n", i, read, vlsensor.channel);
  }

      //Continuous mode logic
      if (vlsensor.sensorMode == 0) {
        //check threshold
        if (read < vlsensor.threshold) {
          //publish the range and the sensor number to correctly pass it to the correct topic
          publishVL6180XRange(read, i);
       //   Serial.printf("Sensor %d triggered\n", i);
        }
      } //Edge mode
      else if (vlsensor.sensorMode == 1) {
        //If state is 0 then sensor is

        if (vlsensor.state == 0) {
          if (read < vlsensor.threshold) {
            //publish the range and the sensor number to correctly pass it to the correct topic
            publishVL6180XRange(read, i);
           // Serial.printf("Sensor %d triggered\n", i);
            vl6180xsensors[i].state = 1; //Sensor is triggered
          }
        } else if (vlsensor.state == 1) {
          //Serial.printf("Sensor %d ############################################ state: %d\n ", i, vlsensor.state);
          if (read > vlsensor.threshold) {

            //publish the range and the sensor number to correctly pass it to the correct topic
            publishVL6180XRange(read, i);
           // Serial.printf("Sensor %d UNtriggered\n", i);
            vl6180xsensors[i].state = 0; //Sensor is triggered
          }
        }
      }
    }

    OPT3001SENSOR optsensor = opt3001sensors[i];
    if (optsensor.enabled) {
      mux.setPort(optsensor.channel);
//delay(1);
      OPT3001 result = opt3001.readResult();
      float lux = result.lux;
      if (optsensor.state)
  {
      Serial.printf("Sensor OPT %d: %f on Port: %d         #####TRIGGERED#####\n", i, lux, optsensor.channel);
  }else{
      Serial.printf("Sensor OPT %d: %f on Port: %d  \n", i, lux, optsensor.channel);
  }
    //  Serial.println(lux);

      //Continuous mode logic
      if (optsensor.sensorMode == 0) {
        //check threshold
        if (lux < optsensor.threshold) {
          //publish the range and the sensor number to correctly pass it to the correct topic
          publishOPT3001Lux(lux, i);
         //  Serial.printf("Sensor %d triggered\n", i);
        }
      } //Edge mode
      else if (optsensor.sensorMode == 1) {
        //If state is 0 then sensor is


        if (optsensor.state == 0) {
          if (lux < optsensor.threshold) {
            //publish the range and the sensor number to correctly pass it to the correct topic
            publishOPT3001Lux(lux, i);
         //    Serial.printf("Sensor %d triggered\n", i);
            opt3001sensors[i].state = 1; //Sensor is triggered
          }
        } else if (optsensor.state == 1) {
      //    Serial.printf("Sensor %d ############################################ state: %d\n ", i, optsensor.state);
          if (lux > optsensor.threshold) {
            //publish the range and the sensor number to correctly pass it to the correct topic
            publishOPT3001Lux(lux, i);
        //     Serial.printf("Sensor %d UNtriggered\n", i);
            opt3001sensors[i].state = 0; //Sensor is triggered
          }
        }
      }
    }

  }
   Serial.println(" ");
  Serial.println(" ");
   Serial.println("################################################ ");
  Serial.println(" ");
   Serial.println(" ");
}

void checkReader()
{
  //Serial.print("Check reader");
  // put your main code here, to run repeatedly:
  const uint16_t flag = 0; //flag configured for Standard inventory
  CAENRFIDTagList *tags, *aux;
  uint16_t numTags;
  char epcStr[2*MAX_ID_LENGTH + 1];
  tags = NULL;
  numTags = 0;

  //run one inventory round
  ec = CAENRFID_InventoryTag(&reader, source, 0, 0, 0, NULL, 0, flag,
                          &tags, &numTags);

  //Serial.printf("Tags detected : %d\n", numTags);
  //Print epc code of detected tags
  while(tags != NULL)
  {
    printHex(tags->Tag.ID, tags->Tag.Length, epcStr);
    Serial.println(" ");
    Serial.println("###############   TAGS   #################### ");
    Serial.printf("  %s\n", epcStr);
    Serial.println("############################################# ");
    Serial.println(" ");
    aux = tags;
    tags = tags->Next;
    free(aux);
    publishTagRead(epcStr);
  }
}

//Setup functions'
void initSettings(char* message) {
  Serial.println(message);
}

void initReaderSettings(char* message) {
  //Parse payload
  
  LogMessage msg;
  
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, message);
  if (error)
    Serial.println(F("Failed to parse message, using default configuration"));

  int rfpower = doc["rf_power"];
  digitalWrite(RFID_RESET_PIN, LOW);
  delay(100);
  digitalWrite(RFID_RESET_PIN, HIGH);
  delay(500);
  ec = CAENRFID_Connect(&reader, CAENRFID_RS232, &port_params);
  // onError(ec);

  // if (ec != CAENRFID_StatusOK)
  // {
  //   msg.code = ReaderInitError;
  //   msg.message = "ConnectionError";
  //   return msg;
  // }
 

  //read reader model and serial number
  ec = CAENRFID_GetReaderInfo(&reader, model, serial);
  // onError(ec);
  printf("Reader info : %d, %s, %s\n", ec, model, serial);

  // if (ec != CAENRFID_StatusOK)
  // {
  //   msg.code = ReaderInitError;
  //   msg.message = "GetReaderInfo error";
  //   return msg;
  // }

  //configure RF power level
  ec = CAENRFID_SetPower(&reader, rfpower);
  // onError(ec);
  printf("Set power to %d mW : %d\n", rfpower, ec);

  // if (ec != CAENRFID_StatusOK)
  // {
  //   msg.code = ReaderInitError;
  //   msg.message = "SetPower error";
  //   return msg;
  // }

  // msg.code = Success;
  // msg.message = "Reader Init Success";
  // return msg;
}

void postComplete(char * message) {
  
  
  if (strcmp(message, "true") == 0)
  {
    setup_finished = true;
    publishLog(LOG_POST, "All Sensor initialization completed", "E0008");
  }
  Serial.print("message: ");
  Serial.print(message);
  Serial.print("   setup_finished: ");
  Serial.println(setup_finished);
}

void reboot() {
  //log reboot request
  publishLog(LOG_INFORMATIONAL, "Reboot Requested", "E0009");

 // digitalWrite(35, LOW);
 // delay(25);
  //digitalWrite(35, HIGH);
 //delay(25);

  for (int i = 0; i < 8; i = i+2) {
    mux.setPort(i);
  vl6180x.stopContinuous();
  Serial.printf("Stopped continuous mode %d", i);
  }
Serial.println("Rebooting");  
  ESP.restart();
}

void initVL6180X_Sensor(char * message, int sensorNum) {
   Serial.println(message);

  //Parse payload
  StaticJsonDocument<512> doc;

  DeserializationError error = deserializeJson(doc, message);
  if (error)
    Serial.println(F("Failed to parse message, using default configuration"));

  bool enabled = doc["enabled"];
  uint8_t address = doc["address"];
  uint8_t channel = doc["channel"];
  uint8_t base = doc["threshold"];
  uint8_t sensorMode = doc["mode"];

  if (enabled) {
    //Set Channel
    mux.setPort(channel);
delay(100);
    //format initialization string


    Serial.print("Initializing VL6180X on channel ");
    Serial.println(channel);

    //Initialize sensor
    vl6180x.init();

    vl6180x.configureDefault();
    // vl6180x.stopContinuous();
    //vl6180x.startRangeContinuous(50);
    vl6180x.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    vl6180x.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
    vl6180x.setTimeout(500);
    //vl6180x.startRangeContinuous(50);
   // vl6180x.stopContinuous();
   //AARON REMOVED
    //delay(250);
    //vl6180x.startInterleavedContinuous(100);
    
   //Initial Readout
    //uint16_t range = vl6180x.readRangeContinuousMillimeters();
    uint16_t read = vl6180x.readRangeSingleMillimeters();
     if (vl6180x.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.println("VL6180X Initial readout ");
    Serial.print("Initial Range:");
    Serial.println(read);
    std::string initString = "Initializing VL6180X sensor on channel " + channel;
    publishLog(LOG_POST, initString.c_str(), "E0010");
    std::string completeString = "VL6180X initialization completed on Sensor " + sensorNum;
    publishLog(LOG_POST, completeString.c_str(), "E0011");
    Serial.println("VL6180X POST completed");

    struct VL6180XSENSOR sensor = {
      .enabled = true,
      .sensorNum = sensorNum,
      .channel = channel,
      .threshold = base,
      .sensorMode = sensorMode,
      .state = 0
      };
    vl6180xsensors[sensorNum] = sensor;
    Serial.print("Sensor ");
    Serial.print(sensorNum);
Serial.println(vl6180xsensors[sensorNum].enabled);
Serial.printf("Sensor %d Mode %d", sensorNum, vl6180xsensors[sensorNum].sensorMode);
  
  }
}

void initOpt3001_Sensor(char* message, int sensorNum) {
  Serial.println(message);

  //Parse payload
  StaticJsonDocument<512> doc;

  DeserializationError error = deserializeJson(doc, message);
  if (error)
    Serial.println(F("Failed to parse message, using default configuration"));

  bool enabled = doc["enabled"];
  uint8_t address = doc["address"];
  uint8_t channel = doc["channel"];
  float base = doc["threshold"];
  uint8_t sensorMode = doc["mode"];

  if (enabled) {
    //Set Channel
    mux.setPort(channel);



    //Initialize Sensor
    Serial.print("Initializing OPT3001 on channel ");
    Serial.println(channel);
    Serial.println(address);

    opt3001.begin(address);
    Serial.print("OPT3001 Manufacturer ID");
    Serial.println(opt3001.readManufacturerID());
    Serial.print("OPT3001 Device ID");
    Serial.println(opt3001.readDeviceID());

    OPT3001_Config newConfig;

    newConfig.RangeNumber = B1100;
    newConfig.ConvertionTime = B0;
    newConfig.Latch = B1;
    newConfig.ModeOfConversionOperation = B11;

    OPT3001_ErrorCode errorConfig = opt3001.writeConfig(newConfig);

    Serial.println("OPT3001 Initial readout....");
    OPT3001 result = opt3001.readResult();

    if (result.error == NO_ERROR) {
      Serial.print("result: ");
      Serial.print(result.lux);
      Serial.println(" lux");
    }
    //format initialization string
    std::string initString = "Initializing OPT3001 sensor on channel " + channel;
    publishLog(LOG_POST, initString.c_str(), "E0012");
    std::string completeString = "OPT3001 initialization completed on Sensor " + sensorNum;
    publishLog(LOG_POST, completeString.c_str(), "E0013");
    Serial.println("OPT3001 POST completed");
    struct OPT3001SENSOR sensor = {
      .enabled = true,
      .sensorNum = sensorNum,
      .channel = channel,
      .threshold = base,
      .sensorMode = sensorMode,
      .state = 0
      };
    opt3001sensors[sensorNum] = sensor;
        Serial.print("Sensor ");
    Serial.print(sensorNum);
Serial.println(opt3001sensors[sensorNum].enabled);
Serial.printf("Sensor %d Mode %d", sensorNum, opt3001sensors[sensorNum].sensorMode);
  }
}

//MQTT outbound functions
void publishLog(LogLevel log_level, const char* message, const char* errorcode) {
  StaticJsonDocument<500> doc;

	 //process the tags
  //int length = sizeof(strings);
  //char tags[100] = "";

  //for (int i = 0; i < length; i++) {
  //    strcat(tags, strings[i]);
  //    if (i < length - 1) {
  //      strcat(tags, ", ");  // Add a comma separator between strings
  //    }
  //}				
							    
   //Construct payload
  doc["device_serial_number"] = device_id; //Change this to the correct USN
  doc["log_level"] = LOG_LEVEL_STRINGS[log_level];
  doc["message"] = message;
  doc["device_timestamp"] = millis();
  doc["tags"] = "";
  doc["error_code"] = errorcode;

  String output;
  serializeJson(doc, output);

  mqttClient->publish(LOGS.c_str(), output.c_str());
}

void publishVL6180XRange(uint8_t range, int sensorNum) {
  StaticJsonDocument<200> doc;

  doc["usn"] = "USN"; //Change this to the correct USN
  doc["range"] = range;
  doc["timestamp"] = millis();

  String output;
  serializeJson(doc, output);

  //get corresponding topic based on the sensor number
  const char* topic;
  if (sensorNum == 1) {
    topic = VL6180X_1_RANGE.c_str();
  } else if (sensorNum == 2) {
    topic = VL6180X_2_RANGE.c_str();
  } else if (sensorNum == 3) {
    topic = VL6180X_3_RANGE.c_str();
  } else if (sensorNum == 4) {
    topic = VL6180X_4_RANGE.c_str();
  }

  mqttClient->publish(topic, output.c_str());
}

void publishOPT3001Lux(float lux, int sensorNum) {
  StaticJsonDocument<200> doc;

  doc["usn"] = "USN"; //Change this to the correct USN
  doc["lux"] = lux;
  doc["timestamp"] = millis();

  String output;
  serializeJson(doc, output);

  //get corresponding topic based on the sensor number
  const char* topic;
  if (sensorNum == 1) {
    topic = OPT3001_1_LUX.c_str();
  } else if (sensorNum == 2) {
    topic = OPT3001_2_LUX.c_str();
  } else if (sensorNum == 3) {
    topic = OPT3001_3_LUX.c_str();
  } else if (sensorNum == 4) {
    topic = OPT3001_4_LUX.c_str();
  }

  mqttClient->publish(topic, output.c_str());
}

void publishTagRead(char* epc) {
  StaticJsonDocument<200> doc;

  doc["epc"] = epc;
  doc["timestamp"] = millis();

  String output;
  serializeJson(doc, output);

  mqttClient->publish(TAG_READ.c_str(), output.c_str());
}

//Light functions

void lightControls(char* message)
{
  StaticJsonDocument<200> doc;
  deserializeJson(doc, message);
  const char* command = doc["command"];
  int led_start = doc["led_start"];
  int led_end = doc["led_end"];
  int led_red = doc["led_red"];
  int led_green = doc["led_green"];
  int led_blue = doc["led_blue"];
  unsigned long timeout = doc["timeout"];
  const char* pattern = doc["pattern"];
  if(strcmp(command, "single") == 0) {
  isPatternRunning = false;
  chasePixel = 0;
  theaterChasePixel = 0;
  setSingleLED(led_start, led_red, led_green, led_blue, timeout);
  } else if(strcmp(command, "range") == 0) {
  isPatternRunning = false;
  chasePixel = 0;
  theaterChasePixel = 0;
  setRangeLEDs(led_start, led_end, led_red, led_green, led_blue, timeout);
  } else if(strcmp(command, "pattern") == 0) {
  isPatternRunning = true;
  uint32_t color = strip.Color(led_red, led_green, led_blue);
  if(strcmp(pattern, "chase") == 0) {
  chase(color);
  } else if(strcmp(pattern, "theater") == 0) {
  theaterChase(color);
    }
  }
}

//CAEN Reader functions

int16_t _connect(void **port_handle, int16_t port_type, void *port_params)
{
    RS232_params *params = (RS232_params *)port_params;
    HardwareSerial *serial;

    // This example supports RS232 port type only
    (void)(port_type);
    // This example will ignore flow control
    (void)(params->flowControl);
    // This example will support 8 databits only
    (void)(params->dataBits);

    // Assign HardwareSerial instance based on serialNum
    switch (params->serialNum)
    {
    case 1:
        serial = &Serial1;
        break;
    case 2:
        serial = &Serial2;
        break;
    default:
        return (-1);
    }

    *port_handle = (void *)serial;

    // Begin Serial with the given parameters
    serial->begin(params->baudrate, SERIAL_8N1, 35, 33);
    //serial->print("connected ");
    Serial.println("connected");

    return (0);
}

int16_t _disconnect(void *port_handle)
{
    HardwareSerial *serial = (HardwareSerial *)port_handle;
    serial->end();
    return (0);
}

int16_t _tx(void *port_handle, uint8_t *data, uint32_t len)
{
    HardwareSerial *serial = (HardwareSerial *)port_handle;
    size_t bytes_sent = serial->write(data, len);
    return (bytes_sent == len ? 0 : -1);
}

int16_t _rx(void *port_handle, uint8_t *data, uint32_t len, uint32_t ms_timeout)
{
    HardwareSerial *serial = (HardwareSerial *)port_handle;
    size_t bytes_received = 0;
    uint32_t start = millis();

    while (bytes_received < len && (millis() - start) <= ms_timeout)
    {
        if (serial->available())
        {

            data[bytes_received++] = serial->read();
            //serial->readBytes(data, len);

        }
     
    }

    return (bytes_received == len ? 0 : -1);
   
}

int16_t _clear_rx_data(void *port_handle)
{
    HardwareSerial *serial = (HardwareSerial *)port_handle;
    while (serial->available())
    {
        serial->read();
    }
    return (0);
}

void _enable_irqs(void)
{
}

void _disable_irqs(void)
{
}

uint64_t get_ms_timestamp(void)
{
    return (uint64_t)millis();
}

static void onError(CAENRFIDErrorCodes ec)
{
  char buffer[100];
    if(ec != CAENRFID_StatusOK)
    {
        sprintf(buffer, "Reader Error with code %d",ec);
        printf(" ****** ERROR (%d) ****** \n", ec);
        //return true;
        String errorString(buffer);
        publishLog(LOG_ERROR, errorString.c_str(), "E0014");
    }
    //return false;
}

static void printHex(uint8_t* vect, uint16_t length, char* result)
{
    int i;
    char* r = result;

    for(i = 0; i < length; i++)
    {
        sprintf(r, "%02X", vect[i]);
        r += 2;
    }
}

//Lights - From ChatGPT
void setSingleLED(int index, int red, int green, int blue, unsigned long timeout) {
  uint32_t color = strip.Color(red, green, blue);
  ledStates[index].prevColor = ledStates[index].color;
  ledStates[index].color = color;
  if (timeout != 0)
  {
    ledStates[index].expirationTime = millis() + timeout;
  } else {
    ledStates[index].expirationTime = 0;
  }
  strip.setPixelColor(index, color);
  strip.show();
}

void setRangeLEDs(int start, int end, int red, int green, int blue, unsigned long timeout) {
  uint32_t color = strip.Color(red, green, blue);
  for(int i=start; i<=end; i++) {
  ledStates[i].prevColor = ledStates[i].color;
  ledStates[i].color = color;
  if (timeout != 0)
  {
    ledStates[i].expirationTime = millis() + timeout;
  } else {
    ledStates[i].expirationTime = 0;
  }
  strip.setPixelColor(i, color);
  }
  strip.show();
}

void chase(uint32_t color) {
 unsigned long now = millis();
 if(now - lastChaseUpdate >= chaseInterval) {
  strip.setPixelColor(chasePixel , color); // Draw new pixel
  strip.setPixelColor(chasePixel-4, 0); // Erase pixel a few steps back
  strip.show();
  lastChaseUpdate = now;
  chasePixel++;

  if(chasePixel >= strip.numPixels() + 4) {
    chasePixel = 0;
  }
 }
}

void theaterChase(uint32_t color) {
 unsigned long now = millis();
 if(now - lastTheaterChaseUpdate >= theaterChaseInterval) {
  for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
  strip.setPixelColor(i+theaterChaseCycle, 0); // Turn every third pixel off
  }
  theaterChaseCycle = (theaterChaseCycle + 1) % 3;
  for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
  strip.setPixelColor(i+theaterChaseCycle, color); // Turn every third pixel on
  }
  strip.show();
  lastTheaterChaseUpdate = now;
  theaterChasePixel++;
  if(theaterChasePixel >= strip.numPixels() + 4) {
    theaterChasePixel = 0;
  }
 }
}

