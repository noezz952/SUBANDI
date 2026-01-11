#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <TinyGPS++.h>
#include <HardwareSerial.h>

#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

//==================================
// LETAK PIN - ALL
//==================================
#define DHT_PIN 4        // GPIO 4: DHT11 DATA
#define DHT_TYPE DHT11
#define TRIG_PIN 5       // GPIO 5: HC-SR04 TRIG
#define ECHO_PIN 18      // GPIO 18: HC-SR04 ECHO

//======SETTING LED & ACT==================
#define BUZZER_PIN 19     //GPIO 19 : BUZZER = BANJIR2
#define LED_HIJAU 21     // GPIO 21: LED HIJAU = NORMAL
#define LED_KUNING 22    // GPIO 22: LED KUNING = HUJAN
#define LED_MERAH 23     // GPIO 23: LED MERAH = BANJIR
#define LED_PUTIH 15
#define LED_BIRU 2

// GPS serial connection
#define GPS_RX 16  // ESP32 RX pin
#define GPS_TX 17  // ESP32 TX pin

//=====SETTING WIFI====================
const char* ssid = "Milky Way";
const char* password = "dikycahya";
const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient mqttclient(espClient);

// topic untuk kirim data sensor ke Colab
const char* sensorTopic = "perseus/iot/sensor/data";

// topic untuk menerima output ML dari Colab
const char* outputTopic = "perseus/iot/sensor/output";

//======SETTING DHT================
DHT dht(DHT_PIN, DHT_TYPE);
String lastStatus;

// inisialisasi Bot Token
#define BOTtoken "8204607669:AAHsFbWae_jRYCtwIiG77_BMRNg-UPjt51w"  // Bot Token dari BotFather

// chat id dari @myidbot
#define CHAT_ID "1822403092"

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

WiFiClientSecure Tele_client;
UniversalTelegramBot bot(BOTtoken, Tele_client);

int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

//=====Variable============
float hum;
float suhu;
float jarak;
float jarak_sebelum = 0;
float jarak_beda;

//===========BOT TELE=============
void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i=0; i<numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    String text = bot.messages[i].text;
    Serial.println(text);

    String from_name = bot.messages[i].from_name;
    while(GPSSerial.available()) {
    gps.encode(GPSSerial.read());
    }

  if (gps.charsProcessed() > 10) {
    float currentLat = gps.location.lat();
    float currentLng = gps.location.lng();

  if (text == "/start") {
      String control = "Selamat Datang, " + from_name + ".\n";
      control += "Gunakan Commands Di Bawah Untuk Monitoring Lokasi GPS\n\n";
      control += "/Lokasi Untuk Mengetahui lokasi saat ini \n";
      bot.sendMessage(chat_id, control, "");
    }

      
    if (text == "/Lokasi"){
      String lokasi = "Lokasi : https://www.google.com/maps/@";
      lokasi +=String(currentLat,6);
      lokasi +=",";
      lokasi +=String(currentLng,6);
      lokasi +=",21z?entry=ttu";
      bot.sendMessage(chat_id, lokasi, "");
  }  
 
   }

  }
}


//=====SET CALLBACK===============
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i=0; i<length; i++) msg += (char)payload[i];
  String status;
  Serial.print("Output from ML: ");
  Serial.println(msg);
  while(GPSSerial.available()) {
    gps.encode(GPSSerial.read());
    }

  if (msg.indexOf("banjir2") != -1) {
    Serial.println( "Banjir2");
    status = "BAHAYA";
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_MERAH, HIGH);
    digitalWrite(LED_KUNING, LOW);
  }
  else if(msg.indexOf("banjir") != -1) {
    Serial.println( "Banjir");
    status = "WASPADA";
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_MERAH, HIGH);
    digitalWrite(LED_KUNING, LOW);
  }
  else if(msg.indexOf("hujan") != -1) {
    Serial.println( "Hujan");
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_MERAH, LOW);
    digitalWrite(LED_KUNING, HIGH);
  }
  else {
    status = "AMAN";
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_HIJAU, HIGH);
    digitalWrite(LED_MERAH, LOW);
    digitalWrite(LED_KUNING, LOW);
  }

  float currentLat = gps.location.lat();
  float currentLng = gps.location.lng();

  if (status != lastStatus){
    if (status == "BAHAYA") {
      String lokasi = "Lokasi : https://www.google.com/maps/@";
      lokasi +=String(currentLat,6);
      lokasi +=",";
      lokasi +=String(currentLng,6);
      lokasi +=",21z?entry=ttu";
      bot.sendMessage(
        CHAT_ID,
        "🚨 *BAHAYA!*\nTerdeteksi banjir di daerah kamu dengan jarak " + String(jarak) + " cm di lokasi : " + lokasi,
        "Markdown"
      );
  }
  else if(status == "WASPADA") {
    String lokasi = "Lokasi : https://www.google.com/maps/@";
    lokasi +=String(currentLat,6);
    lokasi +=",";
    lokasi +=String(currentLng,6);
    lokasi +=",21z?entry=ttu";
    bot.sendMessage(
        CHAT_ID,
        "⚠️ *WASPADA*\nTerdeteksi akan banjir di daerah kamu dengan jarak " + String(jarak) + " cm di lokasi : " + lokasi,
        "Markdown"
      );
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
    }
  lastStatus = status;
  }
}

//=====SET CONNECTION=========
void reconnect() {
  while (!mqttclient.connected()) {
    Serial.print("Connecting to MQTT...");
    digitalWrite(LED_BIRU, HIGH);
    if (mqttclient.connect("esp32client12345")) {
      Serial.println("connected!");
      digitalWrite(LED_BIRU, LOW);
      mqttclient.subscribe(outputTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      delay(2000);
    }
  }
}

// ===========MULAI SEMUA PIN===================
void initAllPins() {
  // HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // 3 LED
  pinMode(LED_HIJAU, OUTPUT);
  pinMode(LED_KUNING, OUTPUT);
  pinMode(LED_MERAH, OUTPUT);
  
  // Matikan semua LED awal
  digitalWrite(LED_HIJAU, LOW);
  digitalWrite(LED_KUNING, LOW);
  digitalWrite(LED_MERAH, LOW);
}


void setup() {
  Serial.begin(115200); //serial on
  dht.begin(); //dht on
  initAllPins();//mulai semua pin
  pinMode( LED_PUTIH, OUTPUT);
  pinMode(LED_BIRU, OUTPUT);



  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //wifi on

  #ifdef ESP32
    Tele_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  #endif
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PUTIH, HIGH);
    Serial.print(".");
    delay(500);
  }
  digitalWrite(LED_PUTIH, LOW);
  Serial.println("WiFi Connected!");
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  Serial.println("GPS NEO-6M Test");
  Serial.println("Waiting for GPS signal...");

  mqttclient.setServer(mqtt_server, 1883);
  mqttclient.setCallback(callback);
  reconnect();
}

//=========SENSOR SR04 (Level Air)=============
float bacaJarakAir() {
  // Kirim pulse 10μs ke TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Baca ECHO dengan timeout 30ms
  long durasi = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Jika timeout, return error
  if (durasi == 0) {
    return -1.0;
  }
  
  // Hitung jarak dalam cm
  // Kecepatan suara = 340 m/s = 0.034 cm/μs
  float jarak_cm = durasi * 0.034 / 2;
  
  // Validasi jarak (HC-SR04 range 2-400cm)
  if (jarak_cm > 400 || jarak_cm < 2) {
    return -1.0;
  }
  
  return jarak_cm;
}


void loop() {
  if (!mqttclient.connected()) reconnect();
  mqttclient.loop();
  
  hum = dht.readHumidity(); //kelembapan
  suhu = dht.readTemperature();//suhu Celcius
  jarak = bacaJarakAir();//jarak ke atas
  jarak_beda = jarak - jarak_sebelum;
  jarak_sebelum = jarak;
  
  if (isnan(hum) || isnan(suhu) || isnan(jarak)) return;

  // bentuk data JSON yang sesuai dengan Python ML
  String jsonData = "{\"hum\": " + String(hum, 2) + ", \"suhu\": " + String(suhu, 2)+ ", \"jarak\": " + String(jarak, 2) + ", \"beda\": " + String(jarak_beda, 2) + "}";

  mqttclient.publish(sensorTopic, jsonData.c_str());
  Serial.println("Published: " + jsonData);
  Serial.println("jarak sekarang : " + String(jarak,2) + "jarak sebelum : " + String(jarak_sebelum,2) + "jarak sekarang : " + String(jarak_beda, 2)); 
  

  
  
  
  

  if (millis() > lastTimeBotRan + botRequestDelay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while(numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }

  delay(2000);//jeda baca
}

