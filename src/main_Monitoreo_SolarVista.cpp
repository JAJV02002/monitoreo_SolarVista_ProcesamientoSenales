//-----PROJECT TITLE: DEVELOPMENT OF A VIRTUAL PILOT PLANT FOR THE MONITORING AND MANAGEMENT OF PHOTOVOLTAIC ENERGY BASED ON THE INTERNET OF THINGS (IoT)----------------
//-----UNIVERSITY OF COLIMA/ AUTONOMOUS UNIVERSITY OF SAN LUIS POTOSI------

//----- CODE FOR THE SIGNAL PROCESSING OF THE VOLTAGE AND CURRENT SENSORS OF THE PHOTOVOLTAIC SYSTEM AND THE SENDING OF DATA TO THE DATABASE----------------

//-----CODE DEVELOPED BY: ENG. JOSÉ ANTONIO JUÁREZ VELÁZQUEZ--------------------------
//-----RESEARCHERS IN CHARGE: JANETH AURELIA ALCALÁ RODRÍGUEZ PhD.---------------------
//-----                       VÍCTOR MANUEL CÁRDENAS GALINDO PhD.----------------------------
//----- last update: DECEMBER 2024-------------

#include <Arduino.h>
#include <WiFi.h>             // Library to connect to WiFi
#include <WiFiClientSecure.h> // Library to make HTTPS requests
#include <FirebaseESP32.h>    // Library to connect ESP32 with Google Firebase
#include <vector>             // Library to work with vectors
#include <utility>            // Library to work with pairs

//****--DEFINITION OF INPUT PINS---*********
const int voltagePin = 34;   // GPIO34 for the voltage sensor
const int currentPin = 35;   // GPIO35 for the current sensor

// Calibration parameters for the ESP32 ADC
const float vRef = 3300.0;      // Reference voltage in mV (3.3V)
const int adcResolution = 4095; // ADC resolution (12 bits)

// Calibration parameters for sensors
const float VCAL = 179.0 / vRef; // Voltage calibration scale according to the maximum input value (127V RMS)
const float ICAL = 15 / vRef;    // Current calibration scale according to the maximum input value of the sensor (15 A RMS)
const unsigned int ADC_COUNTS = adcResolution;

// Variables for the low-pass filter
float offsetV = ADC_COUNTS / 3.1; // Initial voltage offset
float offsetI = ADC_COUNTS / 2.9; // Initial current offset

// Variables for RMS and average calculation
float avgV = 0.0;                // Recursive average value for voltage
float avgI = 0.0;                // Recursive average value for current
float sumSqV = 0.0;              // Cumulative sum of voltage squares
float sumSqI = 0.0;              // Cumulative sum of current squares
unsigned int numSamples = 1000;  // Number of samples for calculation

// Variables to average multiple measurements
const int numAverages = 5;
float vrmsAccumulator = 0.0;
float irmsAccumulator = 0.0;
float vavgAccumulator = 0.0;
float iavgAccumulator = 0.0;
int averageCounter = 0;

// Variables for energy and power
float activePower = 0.0;
float apparentPower = 0.0;
float energyConsumption = 0.0;
unsigned long previousTime = 0;

// Structure to store WiFi network credentials
struct WiFiInfo
{
  const char *ssid;
  const char *password;
};
// Known WiFi networks
const WiFiInfo knownNetworks[] PROGMEM = {
    {"Lab_CEECM", "Lab_CEECM_2024"},
    {"LABCEECM1", "2016labCEECM"},
    {"LABCEECM2", "2016labCEECM"},
    {"Lab_Pot_Ghetto", "labpotlabpot"},
    {"Kawai", "nihongo2024!*"}};

// Variables for Firebase connection
#define FIREBASE_HOST "https://monitorfv-basedatos-esp32-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "1vD8ukIX6yFk1lX3Y6RaYKb7yRDvltUUnnpWqelw"
FirebaseConfig config;
FirebaseAuth auth;
FirebaseData MonitorFV;
const char *ruta = "/Ejemplo_01"; // Changed to const char* to save space

// Function prototypes

//-----------------------------------------------------------------------------------------------------
//****---FUNCTION TO CONNECT TO WIFI---**********
//-----------------------------------------------------------------------------------------------------
void connectWiFi()
{
  for (size_t i = 0; i < sizeof(knownNetworks) / sizeof(knownNetworks[0]); ++i)
  {
    WiFiInfo network;
    memcpy_P(&network, &knownNetworks[i], sizeof(WiFiInfo));
    WiFi.begin(network.ssid, network.password);

    for (int j = 0; j < 5 && WiFi.status() != WL_CONNECTED; j++)
    { // Reduced to 5 attempts
      delay(1000);
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      break;
    }
  }
}

//-----------------------------------------------------------------------------------------------------
//****---FUNCTION TO CONFIGURE FIREBASE---**********
//-----------------------------------------------------------------------------------------------------
void configureFirebase()
{
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
}

//-----------------------------------------------------------------------------------------------------
//****---FUNCTION TO SEND DATA TO MYSQL DATABASE---**********
//-----------------------------------------------------------------------------------------------------
void sendToServer(double voltage, double current, double power, double energy)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFiClientSecure client;
    client.setInsecure(); // Use insecure connection (no certificate validation)

    if (client.connect("monitorFV.com", 443))
    {                 // 443 is the port for HTTPS
      char data[150]; // Buffer to save space with String
      snprintf(data, sizeof(data), "voltaje=%.2f&corriente=%.3f&potencia=%.2f&energia=%.3f",
               voltage, current, power, energy);

      String postData = "POST /save_data_p1.php HTTP/1.1\r\n";
      postData += "Host: monitorFV.com\r\n";
      postData += "Content-Type: application/x-www-form-urlencoded\r\n";
      postData += "Content-Length: ";
      postData += String(strlen(data));
      postData += "\r\n";
      postData += "Connection: close\r\n\r\n";
      postData += data;
      client.print(postData);

      // Skip HTTP headers
      while (client.connected())
      {
        String line = client.readStringUntil('\n');
        if (line == "\r")
          break;
      }
    }
    client.stop();
  }
}

//-----------------------------------------------------------------------------------------------------
//****---SETUP---***************************
//-----------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);     // Initialize serial communication
  analogReadResolution(12); // Set ADC resolution to 12 bits
  previousTime = millis();
  connectWiFi();          // Connect to WiFi network
  configureFirebase();    // Configure Firebase
}

//-----------------------------------------------------------------------------------------------------
//****---MAIN PROGRAM---**********************
//-----------------------------------------------------------------------------------------------------
void loop()
{
  // Initialize sums for RMS calculation
  sumSqV = 0.0;
  sumSqI = 0.0;
  avgV = 0.0;
  avgI = 0.0;

  // Read samples for RMS and average calculation
  for (unsigned int n = 0; n < numSamples; n++)
  {
    int adcReadingVoltage = analogRead(voltagePin);
    int adcReadingCurrent = analogRead(currentPin);

    // Low-pass filter to remove offset
    offsetV += (adcReadingVoltage - offsetV) / 512.0;
    float filteredV = adcReadingVoltage - offsetV;
    offsetI += (adcReadingCurrent - offsetI) / 512.0;
    float filteredI = adcReadingCurrent - offsetI;

    // Calculate the square of the filtered signal
    float sqV = filteredV * filteredV;
    float sqI = filteredI * filteredI;

    // Accumulate squares for RMS
    sumSqV += sqV;
    sumSqI += sqI;

    // Accumulate for average calculation
    avgV += filteredV;
    avgI += filteredI;
  }

  // Calculate RMS and average
  float rmsV = sqrt(sumSqV / numSamples);
  float rmsI = sqrt(sumSqI / numSamples);
  float Vavg = avgV / numSamples;
  float Iavg = avgI / numSamples;

  // Scale RMS and average values
  float Vrms = (rmsV * VCAL * (vRef / 1000.0));
  float Irms = (rmsI * ICAL * (vRef / 1000.0));
  Vavg = Vavg * VCAL * (vRef / 1000.0);
  Iavg = Iavg * ICAL * (vRef / 1000.0);
  
  // Scaling according to voltage ranges
  std::vector<std::pair<double, double>> voltageRanges = {
      {3, 0}, {5, 0}, {7, 0.7185}, {8.5, 0.8}, {10, 1.0391},
      {19.79, 1.007}, {29.76, 1.0192}, {39.72, 1.0311}, {49.68, 1.0286},
      {59.64, 1.0310}, {69.6, 1.0339}, {79.56, 1.0366}, {89.52, 1.0367},
      {99.48, 1.0391}, {109.44, 1.0420}, {119.4, 1.0192}, {127, 1.0391},
      {std::numeric_limits<double>::max(), 1.0414} // For Vrms > 127
  };
  
  for (const auto& range : voltageRanges) {
      if (Vrms <= range.first) {
          Vrms *= range.second;
          break;
      }
  }
  
 // Scaling according to current ranges
  std::vector<std::pair<double, double>> currentRanges = {
      {0.061, 0.00}, {0.123, 0.00}, {0.245, 0.00}, {0.320, 0.0}, {0.42, 0.632}, {0.609, 0.89}, 
      {0.849518, 0.95}, {0.981, 0.8933}, {1.101482, 0.9769}, {1.472, 0.9033}, 
      {1.621166, 0.9937}, {1.964, 0.9088}, {2.153876, 1.0010}, {2.471, 0.9325}, 
      {2.699708, 0.9942}, {2.96, 0.9388}, {3.235178, 0.9965}, {3.463, 0.9439}, 
      {3.784516, 0.9954}, {3.961, 0.9483}, {4.328494, 0.9969}, {4.463, 0.9519}, 
      {4.86157, 0.9991}, {4.961, 0.9650}, {5.39524, 1}, {5.463, 0.9676}, 
      {5.930988, 1.0083}, {5.961, 0.9689}, {6.195048, 1.0110}, {6.460744, 1.0084}, 
      {6.83121, 1.0067}
    };

  
  for (const auto& range : currentRanges) {
      if (Irms <= range.first) {
          Irms *= range.second;
          break;
      }
  }

  // Calculate apparent power
  apparentPower = Vrms * Irms;

  // Calculate active power (assuming power factor 1, no phase)
  activePower = apparentPower; // For resistive loads (pf = 1)

  // Calculate energy consumption (Wh)
  unsigned long currentTime = millis();
  float elapsedTimeHours = (currentTime - previousTime) / 3600000.0; // Time in hours
  energyConsumption += activePower * elapsedTimeHours;
  previousTime = currentTime; // Update time

  // Accumulate for the average of multiple measurements
  vrmsAccumulator += Vrms;
  irmsAccumulator += Irms;
  vavgAccumulator += Vavg;
  iavgAccumulator += Iavg;
  averageCounter++;

  if (averageCounter == numAverages)
  {
    Vrms = vrmsAccumulator / numAverages;
    Irms = irmsAccumulator / numAverages;
    Vavg = vavgAccumulator / numAverages;
    Iavg = iavgAccumulator / numAverages;

    // Reset accumulators
    vrmsAccumulator = 0.0;
    irmsAccumulator = 0.0;
    vavgAccumulator = 0.0;
    iavgAccumulator = 0.0;
    averageCounter = 0;

    // Rename variables to match database data
    float voltaje = Vrms;
    float corriente = Irms;
    float potencia = apparentPower;
    float energia = energyConsumption;

    // Create the message in JSON format
    /*String json = "{";
    json += "\"Vrms\":" + String(Vrms, 3) + ",";
    json += "\"Irms\":" + String(Irms, 3) + ",";
    json += "\"PotenciaActiva\":" + String(activePower, 3) + ",";
    json += "\"PotenciaAparente\":" + String(apparentPower, 3) + ",";
    json += "\"ConsumoEnergetico\":" + String(energyConsumption, 3) + "}";

    // Send the JSON message through the serial port
    Serial.println(json);*/

    // Send data to Firebase
    if (WiFi.status() == WL_CONNECTED && Firebase.ready())
    {
      FirebaseJson json;
      json.set("/sensorData/Voltaje_RMS", voltaje);
      json.set("/sensorData/Corriente_RMS", corriente);
      json.set("/sensorData/Potencia_RMS", potencia);
      json.set("/sensorData/Consumo_energetico", energia);
      Firebase.updateNode(MonitorFV, ruta, json);
    }

    // Send data to MySQL database
    sendToServer(voltaje, corriente, potencia, energia);

    // Add 7.5s delay for data printing on serial monitor (optional)
    delay(7500);
  }
}