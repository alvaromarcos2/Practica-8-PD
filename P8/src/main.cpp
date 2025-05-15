// Álvaro Marcos Roríguez

// 1 

/*#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("Sistema iniciado. Escribe algo:");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial2.write(c);
    Serial.print(">> Enviado a UART2: ");
    Serial.println(c);
  }

  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.write("<< Recibido de UART2: ");
    Serial.println(c);
  }
}*/


// Optativo 2

/*#include <TinyGPS.h>
#include <HardwareSerial.h>

TinyGPS gps;
HardwareSerial SerialGPS(1); // Usamos UART1 (puedes cambiar los pines si es necesario)

void setup() {
  Serial.begin(115200); // Comunicación con PC
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // Comunicación con GPS, RX=16, TX=17
  Serial.println("Iniciando recepción de datos GPS...");
}

void loop() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // Intentamos recibir datos durante un segundo
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (SerialGPS.available()) {
      char c = SerialGPS.read();
      if (gps.encode(c)) {
        newData = true;
      }
    }
  }

  if (newData) {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }

  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
}*/

// Optativo 3

/*#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Wire.h>

#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial
#define SerialAT Serial1

const char apn[] = "internet"; // Cambiar por APN real
const char gprsUser[] = "";
const char gprsPass[] = "";
const char* broker = "test.mosquitto.org";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

void setup() {
  SerialMon.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, 26, 27); // RX=26, TX=27
  delay(3000);

  SerialMon.println("Iniciando modem...");
  modem.restart();
  
  SerialMon.println("Conectando a red móvil...");
  modem.gprsConnect(apn, gprsUser, gprsPass);

  if (modem.isGprsConnected()) {
    SerialMon.println("Conectado a GPRS");
  } else {
    SerialMon.println("Error de conexión GPRS");
  }

  mqtt.setServer(broker, 1883);
}

void loop() {
  if (!mqtt.connected()) {
    SerialMon.println("MQTT desconectado. Intentando reconectar...");
    if (mqtt.connect("ESP32Client")) {
      SerialMon.println("Conectado a MQTT broker");
      mqtt.subscribe("esp/test");
    } else {
      SerialMon.println("Fallo al conectar MQTT");
      delay(5000);
      return;
    }
  }

  long now = millis();
  static long lastSend = 0;
  if (now - lastSend > 10000) {
    lastSend = now;
    mqtt.publish("esp/test", "Hola desde ESP32 GSM");
    SerialMon.println("Mensaje enviado");
  }

  mqtt.loop();
}*/