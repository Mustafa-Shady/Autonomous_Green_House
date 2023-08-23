#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>
#include <Simpletimer.h>

char auth[] = "IBtUnFMyu39jhDvZU1sgRYcjgSijIPSe";

//Set Wifi and password
char ssid[] = "Honor_8C";
char pass[] = "sa3d1234";

SimpleTimer timer;

int Humiditiy;
int Humidifier;
int DeHumidifier;
int Temp;
int Heating;
int Cooling;
int SM_Pre;
int Irrigation;
int WaterPump;

void myTimerEvent()
{
  Blynk.virtualWrite(V1,millis()/1000);
}

void setup() {
  Serial.begin(9600);
  Blynk.begin(auth,ssid,pass);
  /* Sensor  */
  timer.setInterval(1000L,HumiditiyValue);
  timer.setInterval(1000L,HumidifierVlaue);
  timer.setInterval(1000L,DeHumidifierVlaue);
  timer.setInterval(1000L,TempVlaue);
  timer.setInterval(1000L,HeatingVlaue);
  timer.setInterval(1000L,CoolingVlaue);
  timer.setInterval(1000L,SM_PerVlaue);
  timer.setInterval(1000L,IrrigationVlaue);
  timer.setInterval(1000L,WaterPumpVlaue);
}

void loop() {

  if (Serial.available() == 0)
  {
    Blynk.run();
    timer.run();  //Initiates BlynkTimer
  }
  if(Serial.available()>0){
    String SHumiditiy=Serial.readStringUntil('\r');
     Humiditiy = SHumiditiy.toInt();
    String SHumidifier=Serial.readStringUntil('\r');
     Humidifier = SHumidifier.toInt();
    String SDeHumidifier=Serial.readStringUntil('\r');
     DeHumidifier = SDeHumidifier.toInt();
    String STemp=Serial.readStringUntil('\r');
     Temp = STemp.toInt();
    String SHeating=Serial.readStringUntil('\r');
     Heating = SHeating.toInt();
    String SCooling=Serial.readStringUntil('\r');
     Cooling = SCooling.toInt();
    String SSM_Pre=Serial.readStringUntil('\r');
     SM_Pre = SSM_Pre.toInt();
    String SIrrigation=Serial.readStringUntil('\r');
     Irrigation = SIrrigation.toInt();
    String SWaterPump=Serial.readStringUntil('\r');
     WaterPump = SWaterPump.toInt();
  }
}

void HumiditiyValue()
{
  Blynk.virtualWrite(V6,Humiditiy);
}

void HumidifierVlaue()
{
  Blynk.virtualWrite(V9,Humidifier);
}

void DeHumidifierVlaue()
{
  Blynk.virtualWrite(V8,DeHumidifier);
}

void TempVlaue()
{
  Blynk.virtualWrite(V0,Temp);
}

void HeatingVlaue()
{
  Blynk.virtualWrite(V3,Heating);
}

void CoolingVlaue()
{
  Blynk.virtualWrite(V7,Cooling);
}

void SM_PerVlaue()
{
  Blynk.virtualWrite(V2,SM_Pre);
}

void IrrigationVlaue()
{
  Blynk.virtualWrite(V5,Irrigation);
}

void WaterPumpVlaue()
{
  Blynk.virtualWrite(V4,WaterPump);
}
