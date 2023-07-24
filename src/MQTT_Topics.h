#include <string>


std::string endpoint;
// MQTT Topics - Inbound
std::string SETTINGS;
std::string READER_SETTINGS;
std::string READER_START;
std::string READER_STOP;
std::string POSTCOMPLETE;
std::string REBOOT;
std::string VL6180X_1_SETTING;
std::string VL6180X_2_SETTING;
std::string VL6180X_3_SETTING;
std::string VL6180X_4_SETTING;
std::string OPT3001_1_SETTING;
std::string OPT3001_2_SETTING;
std::string OPT3001_3_SETTING;
std::string OPT3001_4_SETTING;
std::string LIGHTS;

// MQTT Topics - Outbound
std::string INIT;
std::string LOGS;
std::string VL6180X_1_RANGE;
std::string VL6180X_2_RANGE;
std::string VL6180X_3_RANGE;
std::string VL6180X_4_RANGE;
std::string OPT3001_1_LUX;
std::string OPT3001_2_LUX;
std::string OPT3001_3_LUX;
std::string OPT3001_4_LUX;
std::string TAG_READ;
std::string LAST_WILL;