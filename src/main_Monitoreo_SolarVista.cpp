//-----Planta piloto virtual para el monitoreo y gestión de energía fotovoltaica-------------
//-----UNIVERSIDAD DE COLIMA/ UNIVERSIDAD AUTONOMA DE SAN LUIS POTOSI------
//-----ESTUDIANTEs: JOSÉ ANTONIO JUÁREZ VELÁZQUEZ--------------------------
//-----INVESTIGADORES: DRA. JANETH AURELIA ALCALÁ RODRÍGUEZ 
//-----                DR. VÍCTOR MANUEL CÁRDENAS GALINDO----------------------------
//---- última actualización: OCTUBRE 2024-------------

#include <Arduino.h>
#include <WiFi.h> // Librería para conectar a WiFi
#include <WiFiClientSecure.h> // Librería para realizar peticiones HTTPS
#include <FirebaseESP32.h> // Librería para conectar ESP32 con Google Firebase

//****--DEFINICION DE PINES DE ENTRADA---*********
const int pinTension = 34;  // GPIO34 para el sensor de tensión
const int pinCorriente = 35; // GPIO35 para el sensor de corriente

// Parámetros de calibración para el ADC del ESP32
const float vRef = 3300.0; // Voltaje de referencia en mV (3.3V)
const int resolucionADC = 4095; // Resolución del ADC (12 bits)

// Parámetros de calibración para sensores
const float VCAL = 179.0 / vRef;  // Escala de calibración de tensión según el valor máximo de entrada de entrada(127V RMS)
const float ICAL = 15 / vRef;  // Escala de calibración de corriente según el valor máximo de entrada del sensor(15 A RMS)
const unsigned int ADC_COUNTS = resolucionADC;

// Variables para el filtro de paso bajo
float offsetV = ADC_COUNTS / 3.1; // Offset de tensión inicial
float offsetI = ADC_COUNTS / 2.9; // Offset de corriente inicial

// Variables para el cálculo del RMS y promedio
float avgV = 0.0; // Valor promedio recursivo para tensión
float avgI = 0.0; // Valor promedio recursivo para corriente
float sumSqV = 0.0; // Suma acumulativa de los cuadrados de tensión
float sumSqI = 0.0; // Suma acumulativa de los cuadrados de corriente
unsigned int numMuestras = 1000; // Número de muestras para cálculo

// Variables para promediar múltiples mediciones
const int numPromedios = 5;
float acumuladorVrms = 0.0;
float acumuladorIrms = 0.0;
float acumuladorVavg = 0.0;
float acumuladorIavg = 0.0;
int contadorPromedios = 0;

// Variables para energía y potencia
float potenciaActiva = 0.0;
float potenciaAparente = 0.0;
float consumoEnergetico = 0.0;
unsigned long tiempoAnterior = 0;

// Estructura para almacenar credenciales de red WiFi
struct WiFiInfo {
  const char* ssid;
  const char* password;
};
// Redes WiFi conocidas
const WiFiInfo knownNetworks[] PROGMEM = {
  {"LABCEECM1", "2016labCEECM"},
  {"LABCEECM2", "2016labCEECM"},
  {"Lab_Pot_Ghetto", "labpotlabpot"},
  {"Kawai", "nihongo2024!*"}
};

// Variables para la conexión a Firebase
#define FIREBASE_HOST "https://monitorfv-basedatos-esp32-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "1vD8ukIX6yFk1lX3Y6RaYKb7yRDvltUUnnpWqelw"
FirebaseConfig config;
FirebaseAuth auth;
FirebaseData MonitorFV;
const char* ruta = "/Ejemplo_01";  // Cambiado a const char* para ahorrar espacio

// Prototipos de funciones

//-----------------------------------------------------------------------------------------------------
//****---FUNCIÓN PARA CONECTAR A WIFI---**********
//-----------------------------------------------------------------------------------------------------
void conectarWiFi() {
  for (size_t i = 0; i < sizeof(knownNetworks) / sizeof(knownNetworks[0]); ++i) {
    WiFiInfo network;
    memcpy_P(&network, &knownNetworks[i], sizeof(WiFiInfo));
    WiFi.begin(network.ssid, network.password);

    for (int j = 0; j < 5 && WiFi.status() != WL_CONNECTED; j++) {  // Reducido a 5 intentos
      delay(1000);
    }
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
  }
}

//-----------------------------------------------------------------------------------------------------
//****---FUNCIÓN PARA CONFIGURAR FIREBASE---**********
//-----------------------------------------------------------------------------------------------------
void FirebaseConfiguracion() {
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
}

//-----------------------------------------------------------------------------------------------------
//****---FUNCIÓN PARA ENVIAR DATOS A BASE DE DATOS MYSQL---**********
//-----------------------------------------------------------------------------------------------------
void sendToServer(double voltaje, double corriente, double potencia, double energia) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClientSecure client;
    client.setInsecure(); // Use insecure connection (no certificate validation)
    
    if (client.connect("monitorFV.com", 443)) { // 443 is the port for HTTPS
      char data[150];  // Buffer para ahorrar espacio con String
      snprintf(data, sizeof(data), "voltaje=%.2f&corriente=%.3f&potencia=%.2f&energia=%.3f", 
               voltaje, corriente, potencia, energia);

      String postData = "POST /save_data_p1.php HTTP/1.1\r\n";
      postData += "Host: monitorFV.com\r\n";
      postData += "Content-Type: application/x-www-form-urlencoded\r\n";
      postData += "Content-Length: ";
      postData += String(strlen(data));
      postData += "\r\n";
      postData += "Connection: close\r\n\r\n";
      postData += data;
      client.print(postData);
      
      // Saltar las cabeceras HTTP
      while (client.connected()) {
        String line = client.readStringUntil('\n');
        if (line == "\r") break;
      }
    }
    client.stop();
  }
}

//-----------------------------------------------------------------------------------------------------
//****---SETUP---***************************
//-----------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); // Inicializar comunicación serial
  analogReadResolution(12); // Configurar resolución del ADC a 12 bits
  tiempoAnterior = millis();
  conectarWiFi(); // Conectar a red WiFi
  FirebaseConfiguracion(); // Configurar Firebase
}

//-----------------------------------------------------------------------------------------------------
//****---PROGRAMA PRINCIPAL---**********************
//-----------------------------------------------------------------------------------------------------
void loop() {
  // Inicializar las sumas para el cálculo del RMS
  sumSqV = 0.0;
  sumSqI = 0.0;
  avgV = 0.0;
  avgI = 0.0;

  // Lectura de muestras para cálculo del RMS y promedio
  for (unsigned int n = 0; n < numMuestras; n++) {
    int lecturaADC_Tension = analogRead(pinTension);
    int lecturaADC_Corriente = analogRead(pinCorriente);

    // Filtro paso bajo para eliminar el offset
    offsetV += (lecturaADC_Tension - offsetV) / 512.0;
    float filteredV = lecturaADC_Tension - offsetV;
    offsetI += (lecturaADC_Corriente - offsetI) / 512.0;
    float filteredI = lecturaADC_Corriente - offsetI;

    // Cálculo del cuadrado de la señal filtrada
    float sqV = filteredV * filteredV;
    float sqI = filteredI * filteredI;

    // Acumulación de los cuadrados para el RMS
    sumSqV += sqV;
    sumSqI += sqI;

    // Acumulación para el cálculo del promedio
    avgV += filteredV;
    avgI += filteredI;
  }

  // Calcular el RMS y el promedio
  float rmsV = sqrt(sumSqV / numMuestras);
  float rmsI = sqrt(sumSqI / numMuestras);
  float Vavg = avgV / numMuestras;
  float Iavg = avgI / numMuestras;

  // Escalamiento de los valores RMS y promedio
  float Vrms = (rmsV * VCAL * (vRef / 1000.0));
  float Irms = (rmsI * ICAL * (vRef / 1000.0));
  Vavg = Vavg * VCAL * (vRef / 1000.0);
  Iavg = Iavg * ICAL * (vRef / 1000.0);

  // Escalamiento según los rangos de voltaje
  if (Vrms <= 3) {
    Vrms *= 0.4;
  } else if (Vrms <= 5) {
    Vrms *= 0.7185;
  } else if (Vrms <= 7) {
    Vrms *= 0.7185;
  } else if (Vrms <= 8.5) {
    Vrms *= 0.8;
  }  else if (Vrms <= 10) {
    Vrms *= 1.0391;
  } else if (Vrms <= 19.79) {
    Vrms *= 1.007;
  } else if (Vrms <= 29.76) {
    Vrms *= 1.0192;
  } else if (Vrms <= 39.72) {
    Vrms *= 1.0311;
  } else if (Vrms <= 49.68) {
    Vrms *= 1.0286;
  } else if (Vrms <= 59.64) {
    Vrms *= 1.0310;
  } else if (Vrms <= 69.6) {
    Vrms *= 1.0339;
  } else if (Vrms <= 79.56) {
    Vrms *= 1.0366;
  } else if (Vrms <= 89.52) {
    Vrms *= 1.0367;
  } else if (Vrms <= 99.48) {
    Vrms *= 1.0391;
  } else if (Vrms <= 109.44) {
    Vrms *= 1.0420;
  } else if (Vrms <= 119.4) {
    Vrms *= 1.0192;
  } else if (Vrms <= 127) {
    Vrms *= 1.0391;
  } else if (Vrms > 127) {
    Vrms *= 1.0414;
  }

  // Escalamiento según los rangos de corriente
  if (Irms <= 0.061) {
    Irms *= 0.01;
  } else if (Irms <= 0.123) {
    Irms *= 0.3;
  } else if (Irms <= 0.245) {
    Irms *= 0.563;
  } else if (Irms <= 0.493) {
    Irms *= 0.66;
  } else if (Irms <= 0.737) {
    Irms *= 0.8714;
  } else if (Irms <= 0.981) {
    Irms *= 0.8933;
  } else if (Irms <= 1.472) {
    Irms *= 0.9033;
  } else if (Irms <= 1.964) {
    Irms *= 0.9088;
  } else if (Irms <= 2.471) {
    Irms *= 0.9325;
  } else if (Irms <= 2.96) {
    Irms *= 0.9388;
  } else if (Irms <= 3.463) {
    Irms *= 0.9439;
  } else if (Irms <= 3.961) {
    Irms *= 0.9483;
  } else if (Irms <= 4.463) {
    Irms *= 0.9519;
  } else if (Irms <= 4.961) {
    Irms *= 0.9650;
  } else if (Irms <= 5.463) {
    Irms *= 0.9676;
  } else if (Irms <= 5.961) {
    Irms *= 0.9689;
  } else if (Irms <= 6.463) {
    Irms *= 0.9621;
  } else if (Irms <= 6.8) {
    Irms *= 0.9139;
  }

  // Cálculo de la potencia aparente
  potenciaAparente = Vrms * Irms;

  // Cálculo de la potencia activa (suponiendo factor de potencia 1, sin fase)
  potenciaActiva = potenciaAparente;  // Para cargas resistivas (pf = 1)

  // Cálculo del consumo energético (Wh)
  unsigned long tiempoActual = millis();
  float tiempoTranscurridoHoras = (tiempoActual - tiempoAnterior) / 3600000.0;  // Tiempo en horas
  consumoEnergetico += potenciaActiva * tiempoTranscurridoHoras;
  tiempoAnterior = tiempoActual;  // Actualizar el tiempo

  // Acumulación para el promedio de múltiples mediciones
  acumuladorVrms += Vrms;
  acumuladorIrms += Irms;
  acumuladorVavg += Vavg;
  acumuladorIavg += Iavg;
  contadorPromedios++;

  if (contadorPromedios == numPromedios) {
    Vrms = acumuladorVrms / numPromedios;
    Irms = acumuladorIrms / numPromedios;
    Vavg = acumuladorVavg / numPromedios;
    Iavg = acumuladorIavg / numPromedios;

    // Resetear acumuladores
    acumuladorVrms = 0.0;
    acumuladorIrms = 0.0;
    acumuladorVavg = 0.0;
    acumuladorIavg = 0.0;
    contadorPromedios = 0;

    // Cambio de nombre de variables para que coincidan con los datos de la base de datos
    float voltaje = Vrms;
    float corriente = Irms;
    float potencia = potenciaAparente;
    float energia = consumoEnergetico;

    // Crear el mensaje en formato JSON
    /*String json = "{";
    json += "\"Vrms\":" + String(Vrms, 3) + ",";
    json += "\"Irms\":" + String(Irms, 3) + ",";
    json += "\"PotenciaActiva\":" + String(potenciaActiva, 3) + ",";
    json += "\"PotenciaAparente\":" + String(potenciaAparente, 3) + ",";
    json += "\"ConsumoEnergetico\":" + String(consumoEnergetico, 3) + "}";
    
    // Enviar el mensaje JSON por el puerto serial
    Serial.println(json);*/

    // Enviar los datos a Firebase
    if (WiFi.status() == WL_CONNECTED && Firebase.ready()) {
    FirebaseJson json;
    json.set("/sensorData/Voltaje_RMS", voltaje);
    json.set("/sensorData/Corriente_RMS", corriente);
    json.set("/sensorData/Potencia_RMS", potencia);
    json.set("/sensorData/Consumo_energetico", energia);
    Firebase.updateNode(MonitorFV, ruta, json);
  }

    // Enviar los datos a la base de datos MySQL
    sendToServer(voltaje, corriente, potencia, energia);

    // Agregar espera de 7.5s para impresión de datos en monitor serial (opcional)
    delay(7500);
  }
}
//-----------------------------------------------------------------------------------------------------