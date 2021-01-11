/**********************************************************************************************************************************
Sketch Name: Weather_Station_Mega_open_network.ino
Date: 01-06-2018

Board Used: Arduino Mega
IDE used: Arduino 1.8.5

Coder: Ashish Kumar Pardeshi
Deisgnation: Project Scientific Officer - B
Organization: HBCSE, TIFR, Mumbai.

Description: This Sketch is measuring the parameters: light intensity in Lux, Humidity in %,
             Temperature in *C, Atmospheric Pressure in mb, Wind Speed in m/s and Wind Vane (In terms Direction) with Time Stamp.
Interfacings: 
S.no    Module        Interface              No of Pins                      Connector          Used For                                
//---------------- Sensor Modules ------------------------------------------------------------------------------------
1       BH1750        I2C (SDA, SCL)        (Vcc, Gnd, SDA, SCL)             4-pin RJ11        Lux Calculation
2       BMP180        I2C (SDA, SCL)        (Vcc, Gnd, SDA, SCL)             4-pin RJ11        Atmospheric Pressure and Temperature
3       DHT-22        Dig Pin 4             (Vcc, Gnd, Data)                 4-pin RJ11        Humidity and Temperature
4       Anemometer    Dig Pin 2 (INT0)      (Vcc, Gnd, Rotational Pulse)     4-pin RJ11        Wind Speed
5       Wind Vane     Dig Pins              (Vcc, Gnd, N,S,W,E,NW,NE,SW,SE)  10-pin Conn       Wind Direction
                      (5,6, 7, 8)
                      (9, 10, 11) 
                      (12). 
6       Rain Guage    Dig Pin 3             (Vcc, Gnd, Tipping Pulse)        4-pin RJ11        Rain Fall

//---------------- Other Modules -------------------------------------------------------------------------------------
7       DS3231 RTC    I2C (SDA, SCL)        (Vcc, Gnd, SDA, SCL)             (On Board)        Time and Date
8       ESP-01        UART1 (Rx1, Tx1)      (Vcc, Gnd, Rx1, Tx1)             (On Board)        Wifi
        ESP_Switch    (42)                                                   (On Board)
9       Bluetooth     UART0 (Rx2, Tx2)      (Vcc, Gnd, Rx0, Tx0)             (On Board)        Bluetooth  
        Ble_Switch    (44)                                                   (On Board)
                
**************************************************************************************************************************/
//#include <SFE_BMP180.h>
#include "SparkFunBME280.h"
#include "DHT.h"
#include <BH1750.h>         //Lux sensor Lib
#include <WiFiEspClient.h>  
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <PubSubClient.h>

// RTC DS3231 Lib Header
#include <Wire.h>
#include <DS3231.h>   
#include <Time.h> 

//#include "SoftwareSerial.h"

char* WIFI_AP = "HOTSPOT";
//char *WIFI_PASSWORD = "xxxxxxxxxxxxxxx";
int count=0, reconnect_counter = 0;
unsigned signal_avail_global[3];

#define TOKEN "xxxxxxxxxxx"

// DHT
#define DHTPIN 4
#define DHTTYPE DHT22

char thingsboardServer[] = "158.144.43.11"; //"14.139.123.11"; //"10.1.0.90";

// Initialize the Ethernet client object
WiFiEspClient espClient;
#define esp_switch 42

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

PubSubClient client(espClient);

//Serial1(14, 5); // RX, TX (4,5)

int status = WL_IDLE_STATUS;
unsigned long lastSend;

//--------------- RTC DS3231 --------------------------
DS3231  rtc(SDA, SCL);
Time  t;
String date_time_stamp = " ", date_stamp,time_stamp;
unsigned char last_date = 0, last_hour = 0;

//-------------RainFall Macros and Objects-------------
#define rain_tip  3
volatile double rain = 0.00;
volatile double hour_rain = 0.00;
float tip = 0.17139;

//--------- BME280 Macros and Objects------------------;
BME280 mySensor;
#define ALTITUDE 13.0 // Altitude of HBCSE in meters
double T, P, p0, a;

//--------- Lux Macros and Objects---------------------
BH1750 lightMeter;
uint16_t lux = 0;

//--------- Wind Direction Macros and Objects----------
unsigned int last_wind_direction = 0;
String w_direction = "N";
int wind_dir_no = 0;
#define N 5
#define N_W 6
#define W 7
#define S_W 8
#define S 9
#define S_E 10
#define E 11
#define N_E 12

//--------- Wind Speed Macros and Objects--------------
#define windSensor_out_interruptpin  2
const float pi = 3.14159265;          // pi number
unsigned int period = 1000;           // Measurement period (miliseconds)
float radius = 85 * 0.001; //95.2 ;                 // Distance from center windmill to sensor (meter)
float radius_m = 0.00;
volatile unsigned int counter = 0;      // counter for sensor
float windSpeed = 0;                    // Wind speed (m/s)


#define ble_switch 44



void setup()
{
  // initialize Serial2 for debugging
  Serial2.begin(9600);
  pinMode(esp_switch,OUTPUT);
  pinMode(ble_switch,OUTPUT);
  digitalWrite(esp_switch, HIGH);
  digitalWrite(ble_switch,LOW);
  rtc.begin();
  dht.begin();
  lightMeter.begin();
  Wind_Dir_Init();

  Wire.begin();
  mySensor.setI2CAddress(0x76);
  
  if (mySensor.beginI2C() == true)
    Serial2.println("BME280 ok");
  else
  {
    Serial2.println("BME280 fail");
  }

  RTC_data();
  Serial2.println(date_time_stamp);
  
  InitWiFi();
  client.setServer( thingsboardServer, 1883 );
  lastSend = 0;

  rainfall_read();
  pinMode(rain_tip,INPUT);
}

void(*resetFunc)(void) = 0;

void loop()
{
  t = rtc.getTime();
  reconnect_counter = 0;
  status = WiFi.status();
  if ( status != WL_CONNECTED)
  {
    while ( status != WL_CONNECTED)
    {
      if(reconnect_counter >= 0 && reconnect_counter < 3)
      {
       WIFI_AP = WiFi.SSID(signal_avail_global[0] );
      }
      if(reconnect_counter >=3 && reconnect_counter <=5)
      {
        WIFI_AP = WiFi.SSID(signal_avail_global[1] );
      }
      if(reconnect_counter > 5)
      {
        digitalWrite(esp_switch, HIGH);
        resetFunc();
        delay(300);
      }
      Serial2.print("connect to SSID: ");
      Serial2.println(WIFI_AP);
      // Connect to WPA/WPA2 network
      status = WiFi.beginOn(WIFI_AP);
      delay(50);
      reconnect_counter++;
    }
    Serial2.println("Got AP");
  }

  if ( !client.connected() )
  {
    reconnect();
  }

  //if ( millis() - lastSend > 1000 )     // Update and send only after 1 seconds
  //{
  getAndSendTemperatureAndHumidityData();
  delay(10);
  getAndSendAtmosphericPressure();

  windspeed();

  rainfall_data();

  //Wind_Dir_Process();
  //  lastSend = millis();
  //}

  client.loop();
}

// ISR for External Intrerrupt 0
void countPulses()
{
  if (digitalRead(2) == HIGH) //HIGH
  {
    delayMicroseconds(1);
    //while (digitalRead(2) == HIGH); // HIGH// wait while tooth is passing the sensor*/
    counter++;
  }
}

void windspeed()
{
  windSpeed = 0;
  counter = 0;
  attachInterrupt(0/*digitalPinToInterrupt(windSensor_out_interruptpin)*/, countPulses,RISING); //RISING
  unsigned long millis();
  unsigned long startTime = millis();
  while (millis() < startTime + period)
  {

  }
  detachInterrupt(0);//digitalPinToInterrupt(windSensor_out_interruptpin));
  windSpeed = (( 2 * pi * radius * 0.125 * counter) / (period / 1000));  // Calculate wind speed on m/s
  Serial2.print("Wind Speed: ");
  Serial2.print(windSpeed);
  Serial2.println(" m/s");

  String wind_speed = String(windSpeed);

  String payload = "{";
  payload += "\"Wind Speed\":"; payload += wind_speed;
  payload += "}";

  // Send payload
  char attributes[100];
  payload.toCharArray( attributes, 100 );
  client.publish("v1/devices/me/telemetry", attributes );
}

void rainfall_data()
{
  String rain_data = String(rain);
  String rainhour_data = String(hour_rain);

  String payload = "{";
  payload += "\"Rain_hour\":"; payload += rainhour_data; payload += ",";
  payload += "\"Rain_day\":"; payload += rain_data;
  payload += "}";

  // Send payload
  char attributes[100];
  payload.toCharArray( attributes, 100 );
  client.publish("v1/devices/me/telemetry", attributes );
  
  if(t.date != last_date)
  {
    last_date = t.date;
    
    rain = 0.0;
  }

  if(t.hour != last_hour)
  {
    last_hour = t.hour;
    hour_rain = 0.0;
  }
  //Serial2.println("LAst Time");

  //Serial2.println(last_date);
  //Serial2.println(last_hour);
}

void rainfall_read()
{
  attachInterrupt(1/*digitalPinToInterrupt(windSensor_out_interruptpin)*/, rain_count,FALLING); //RISING  
}

void rain_count()
{
  if(digitalRead(rain_tip) == LOW)
  {
    rain += tip;
    hour_rain += tip;
  }
}


void getAndSendAtmosphericPressure()
{
  char status;
  //Serial2.println("Col Atmos Pres.");

  float humbme = mySensor.readFloatHumidity();
  float tempbme = mySensor.readTempC();
  float presbme = (mySensor.readFloatPressure()/100);

  /*//status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      /* Serial2.print("Temp: ");
        Serial2.print(T,2);
        Serial2.println(" *C");*/

     /*// status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          // Print out the measurement:
          /*Serial2.print("absolute pres: ");
            Serial2.print(P,2);
            Serial2.println(" mb");*/

         //// p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
          /*Serial2.print("Sealevel pres: ");
            Serial2.print(p0,2);
            Serial2.println(" mb, ");*/

       ////   a = pressure.altitude(P, p0);
          /*Serial2.print("altitude: ");
            Serial2.print(a,0);
            Serial2.println(" m");*/
       /*// }
        else
        {
          Serial2.println("e1\n");
          return;
        }
      }
      else
      {
        Serial2.println("e2\n");
        return;
      }
    }
    else
    {
      Serial2.println("e3\n");
      return;
    }
  }
  else
  {
    Serial2.println("e4\n");
    return;
  }
*///

  String temperaturebm280 = String(tempbme);
  String humiditybme280 = String(humbme);
  String absolute_pressure = String(presbme);
  //String alti = String(a);

  // Just debug messages
  /* Serial2.print( "Sen abso Value : [" );
    Serial2.print( P); Serial22.print( "," );
    Serial2.print( sealevel_pressure );
    Serial2.print( "]   -> " );*/



  // Prepare a JSON payload string
  String payload = "{";
  //payload += "\"alti\":"; payload += alti; payload += ",";
  payload += "\"temperature_BME280\":"; payload += temperaturebm280; payload += ",";
  payload += "\"humidity_BME280\":"; payload += humiditybme280; payload += ",";
  payload += "\"absolute pressure\":"; payload += absolute_pressure;
  payload += "}";

  // Send payload
  char attributes[300];
  payload.toCharArray( attributes, 300 );
  client.publish("v1/devices/me/telemetry", attributes );
  //Serial2.println( attributes );

}

void getAndSendTemperatureAndHumidityData()
{
  //Serial2.println("Col temp data.");

  // Reading temperature or humidity takes about 250 milliseconds!
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t))
  {
    Serial2.println("Fail DHT");
    return;
  }


  /*Serial22.print("Humidity: ");
    Serial22.print(h);
    Serial22.print(" %\t");
    Serial22.print("Temperature: ");
    Serial22.print(t);
    Serial22.print(" *C ");*/
  String lux = String(lightMeter.readLightLevel());
  String temperature = String(t);
  String humidity = String(h);

  Wind_Dir_Process();
  RTC_data();
  // Just debug messages
  //Serial22.print( "Send Temp & humid: [" );
  //Serial22.print( temperature ); Serial22.print( "," );
  //Serial22.print( humidity );
 // Serial22.print( "]   -> " );
    Serial2.print( "[" );
    Serial2.print(date_time_stamp);
    Serial2.print( "]   -> " );

  // Prepare a JSON payload string
  String payload = "{";
  payload += "\"temperature\":"; payload += temperature; payload += ",";
  payload += "\"lux\":"; payload += lux; payload += ","; 
  payload += "\"humidity\":"; payload += humidity; payload += ",";
  payload += "\"wind_dir\":"; payload += String(wind_dir_no); payload += ",";
  payload += "\"wind direction\":"; payload += w_direction;
  payload += "}";

  // Send payload
  char attributes[100];
  payload.toCharArray( attributes, 100 );
  client.publish("v1/devices/me/telemetry", attributes );
  Serial2.println( attributes );
}

/*void InitWiFi()
{
  
  Serial1.begin(9600);    // initialize 2 for ESP module
  
  WiFi.init(&Serial1);    // initialize ESP module
  
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) 
  {
    Serial2.println("WiFi absent");
    // don't continue
    while (true);
  }

  Serial2.println("Conn to AP ...");
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) 
  {
    Serial2.print("Attemp to connect SSID: ");
    Serial2.println(WIFI_AP);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    delay(500);
  }
  Serial1.println("Connected AP");
}*/

void reconnect()
{
  // Loop until we're reconnected
  unsigned int recounter = 0;
  while (!client.connected())
  {
    Serial2.print("Connect to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect("Arduino Uno Device", TOKEN, NULL) )
    {
      Serial2.println( "[DONE]" );
    }
    else
    {
      if(recounter > 2)
      {
        digitalWrite(esp_switch, HIGH);
        resetFunc();
        delay(300);
      }
      Serial2.print( "[FAILED] [ rc = " );
      Serial2.print( client.state() );
      Serial2.println( " : retry in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
      recounter++;
    }
  }
}

void Wind_Dir_Init()
{
  for (int i = N; i <= N_E; i++)
  {
    pinMode(i, INPUT);
  }
}

void Wind_Dir_Process()
{
  unsigned int wind_dir_read;
  for (unsigned int i = N; i <= N_E; i++)
  {
    if (digitalRead(i) == 0)
    {
      wind_dir_read = i;
      break;
    }
    else
    {
      wind_dir_read = last_wind_direction;
    }
  }
  

  switch (wind_dir_read)
  {
    case 5: w_direction = "W";
      wind_dir_no = 270;
      break;

    case 6: w_direction = "SW";
      wind_dir_no = 225;
      break;

    case 7: w_direction = "S";
      wind_dir_no = 180;
      break;

    case 8: w_direction = "SE";
      wind_dir_no = 135;
      break;

    case 9: w_direction = "E";
      wind_dir_no = 90;
      break;

    case 10: w_direction = "NE";
      wind_dir_no = 45;
      break;

    case 11: w_direction = "N";
      wind_dir_no = 0;
      break;

    case 12: w_direction = "NW";
      wind_dir_no = 315;
      break;

    //default: w_direction = "S";
     // wind_dir_no = 0;

  }
  last_wind_direction = wind_dir_read;

  /*String payload = "{";
    payload += "\"wind_dir\":"; payload += String(wind_dir_no); payload += ",";
    payload += "\"wind direction\":"; payload += w_direction;
    payload += "}";

    // Send payload
    char attributes[100];
    payload.toCharArray( attributes, 100 );
    client.publish("v1/devices/me/telemetry", attributes );
    //Serial2.println( attributes );*/

}

void RTC_data()
{
  t = rtc.getTime();
  date_time_stamp = t.date/10;
  date_time_stamp = date_time_stamp + t.date%10 + "-" + t.mon/10 + t.mon%10 + " " + t.hour/10 + t.hour%10 + ":"+ t.min/10 + t.min%10 + ":" + t.sec/10 + t.sec%10;

  date_stamp = t.date/10;
  date_stamp = date_stamp + t.date%10  + t.mon/10 + t.mon%10 + t.year;
  
  time_stamp = t.hour/10;
  time_stamp = time_stamp + t.hour%10 + ":"+ t.min/10 + t.min%10 + ":" + t.sec/10 + t.sec%10;
  return;
}
