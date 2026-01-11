#include <DHT.h>

//==================================
// LETAK PIN - ALL
//==================================
#define DHT_PIN 4        // GPIO 4: DHT11 DATA
#define DHT_TYPE DHT11

//======SETTING DHT================
DHT dht(DHT_PIN, DHT_TYPE);

//=====Variable============
float hum;
float suhu;

void setup() {
  Serial.begin(115200); //serial on
  dht.begin(); //dht on

}

void loop() {
  // put your main code here, to run repeatedly:
  hum = dht.readHumidity()+10; //kelembapan
  suhu = dht.readTemperature();//suhu Celcius

  //hum = random(81,90);
  //suhu = random(24,26);
  displayData();
  delay(2000);//jeda baca
}

void displayData(){
  Serial.print(hum);
  Serial.print(",");
  Serial.print(suhu);
  Serial.print(",");
  if (suhu < 26 || hum > 70) {
    Serial.println("Dingin");
  }else if (suhu < 31 || hum > 50){
    Serial.println("Normal");
  }else if (suhu >30 || hum < 60){
    Serial.println("Panas");
  }
}
