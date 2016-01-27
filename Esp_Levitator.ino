/*
  magnetic levitation
  Chiangmai Maker Club
  modified -
  by wasin wongkum
  sensor hall sensor A1302
  mosfet irf 540n
  supply 12v
  coil 10 ohm
*/
#include <EEPROM.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define key 0x09

#define buffer_size 512

#define LED        16
#define hall        A0
#define magnetic    15
#define filter      0.90f
#define Start_adddress    1

char data[buffer_size] = {0}; //buffer to hold incoming and outgoing packets
WiFiUDP Udp;

typedef struct
{
  int8_t startByte;
  int8_t roll;
  int8_t pitch;
  int8_t throttle;
  int8_t yaw;
  int8_t checksum;
} ControlData;

typedef struct
{
  int8_t startByte;
  int8_t startByte2;
  int8_t yawPitchRoll;
  int16_t kp;
  int16_t ki;
  int16_t kd;
  int16_t checksum;
} TuningData;



float state = 0;
float state_f = 0;
float prev_state = 0;
float prev_state_f = 0;
float ref = 7.0;
float error = 0;
float prev_error = 0;
float error_dot = 0;
float error_sum = 0;

float kp = 500;
float ki = 20;
float kd = 5;
float output  = 0;

Ticker ticker_PID;
Ticker ticker_DEGUG;

int address = 0;

void Read_UDP();
void PID();
void Debug();

void EEPROM_read_Gain()
{
  int buffer[6] = {0};

  for (int i = 0; (i < 6); i++)
  {
    buffer[i] = EEPROM.read(Start_adddress + i);
  }
  int16_t g_buffer = (int16_t)buffer[0] << 8 | (int16_t)buffer[1];
  kp = (float)g_buffer / 1.0f;

  g_buffer = (int16_t)buffer[2] << 8 | (int16_t)buffer[3];
  ki = (float)g_buffer / 10.0f;

  g_buffer = (int16_t)buffer[4] << 8 | (int16_t)buffer[5];
  kd = (float)g_buffer / 100.0f;
}

void EEPROM_write_Gain()
{
  int buffer[6] = {0};

  int16_t g_buffer = kp * 1.0f;
  buffer[0] =  (int8_t)(g_buffer >> 8);
  buffer[1] =  (int8_t)g_buffer;
  g_buffer = ki * 10.0f;
  buffer[2] =  (int8_t)(g_buffer >> 8);
  buffer[3] =  (int8_t)g_buffer;
  g_buffer = kd * 100.0f;
  buffer[4] =  (int8_t)(g_buffer >> 8);
  buffer[5] =  (int8_t)g_buffer;

  for (int i = 0; (i < 6); i++)
  {
    EEPROM.write(Start_adddress + i, buffer[i]);
  }

  EEPROM.commit();
}

void PRE_EEPROM_write_Gain()
{
  if (EEPROM.read(0) != key)
  {
    EEPROM.write(0, key);

    int buffer[6] = {0};

    int16_t g_buffer = 500 ;
    buffer[0] =  (int8_t)(g_buffer >> 8);
    buffer[1] =  (int8_t)g_buffer;
    g_buffer = 200;
    buffer[2] =  (int8_t)(g_buffer >> 8);
    buffer[3] =  (int8_t)g_buffer;
    g_buffer = 500 ;
    buffer[4] =  (int8_t)(g_buffer >> 8);
    buffer[5] =  (int8_t)g_buffer;

    for (int i = 0; (i < 6); i++)
    {
      EEPROM.write(Start_adddress + i, buffer[i]);
    }

    EEPROM.commit();
  }
}
/* Set these to your desired credentials. */
const char *ssid = "Levitator";
const char *password = "12345678";
unsigned int localPort = 12345;
IPAddress local_ip(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 255);

void setup()
{
  pinMode(hall, INPUT);
  pinMode(output, OUTPUT);
  pinMode(LED, OUTPUT);

  delay(1000);
  Serial.begin(115200);

  char accessPointName[16] = {'\0'};
  sprintf(accessPointName, "Levitator-%lu", ESP.getChipId());
  Serial.print("Access Point: ");
  Serial.println(accessPointName);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  WiFi.softAP(accessPointName, "");
  WiFi.mode(WIFI_AP_STA);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  delay(500);
  // WiFi.softAPConfig(local_ip, gateway, subnet);
  // delay(500);

  // IPAddress myIP = WiFi.softAPIP();
  // Serial.print("AP IP: ");
  // Serial.println(myIP);

  if ( Udp.begin(localPort) > 0) {
    Serial.print("UDP star port :");
    Serial.println(Udp.localPort());

  } else {
    Serial.println("Error");

  }
  delay(1000);

  EEPROM.begin(10);

  PRE_EEPROM_write_Gain();
  EEPROM_read_Gain();

  ticker_PID.attach_ms(10, PID);
  // ticker_DEGUG.attach_ms(1000, Debug);


}

void loop()
{
  Read_UDP();
  delay(5);

}

void Read_UDP()
{

  int numberOfBytes = Udp.parsePacket();

  if (numberOfBytes > 0)
  {
    Udp.read(data, numberOfBytes);

    if (data[0] == 0XF0 && data[1] == 0XF0) // tuning data
    {
      TuningData tuningData = {0};
      memcpy(&tuningData, data, sizeof(TuningData));
      int16_t checksum = tuningData.yawPitchRoll + tuningData.kp + tuningData.ki + tuningData.kd;

      if (tuningData.yawPitchRoll >= 0X01 && tuningData.yawPitchRoll <= 0X03 && tuningData.checksum == checksum)
      {
        switch (tuningData.yawPitchRoll)
        {
          case 0X02: // pitch
            kp = (float)tuningData.kp / 1.0f;
            ki = (float)tuningData.ki / 10.0f;
            kd = (float)tuningData.kd / 100.0f;
            EEPROM_write_Gain();
            break;
        }


          Serial.print("Tuning Data ");
          Serial.print(tuningData.yawPitchRoll == 0X01 ? "(Yaw):" :
                       tuningData.yawPitchRoll == 0X02 ? "(Pitch):" : "(Roll):");
          Serial.print(" KP=");
          Serial.print(kp);
          Serial.print(" KI=");
          Serial.print(ki);
          Serial.print(" KD=");
          Serial.println(kd);
      }
    }
    else if (data[0] == 0XFC && data[0] == 0XFC) // get tuning data
    {
      TuningData tuningData = {0};
      memcpy(&tuningData, data, sizeof(TuningData));
      int8_t checksum = tuningData.yawPitchRoll + tuningData.kp + tuningData.ki + tuningData.kd;

      if (tuningData.checksum == checksum)
      {
        // send tuning data to remote control
        for (tuningData.yawPitchRoll = 0X01; tuningData.yawPitchRoll <= 0X03; tuningData.yawPitchRoll++)
        {
          // sample
          float _kp = 0.0f;
          float _ki = 0.0f;
          float _kd = 0.0f;

          switch (tuningData.yawPitchRoll)
          {
            case 0X01: // yaw
             _kp = 0;
             _ki = 0;
             _kd = 0;
                break;

            case 0X02: // pitch
             _kp = kp;
             _ki = ki;
             _kd = kd;
                break;

            case 0X03: // roll
             _kp = 0;
             _ki = 0;
             _kd = 0;
                break;

          }
          // ************

          tuningData.kp = _kp * 1.0f;
          tuningData.ki = _ki * 10.0f;
          tuningData.kd = _kd * 100.0f;
          tuningData.checksum = tuningData.yawPitchRoll + tuningData.kp + tuningData.ki + tuningData.kd;
          memcpy(&data, &tuningData, sizeof(TuningData));

          Serial.print("Send tuning data to ");
          Serial.print(Udp.remoteIP());
          Serial.print(" port ");
          Serial.print(Udp.remotePort());
          Serial.print(": 0X");
          Serial.print((byte)tuningData.startByte, HEX);
          Serial.print(" 0X");
          Serial.print((byte)tuningData.startByte2, HEX);
          Serial.print(" 0X0");
          Serial.print((byte)tuningData.yawPitchRoll, HEX);
          Serial.print(" ");
          Serial.print(tuningData.kp);
          Serial.print(" ");
          Serial.print(tuningData.ki);
          Serial.print(" ");
          Serial.print(tuningData.kd);
          Serial.print(" checksum ");
          Serial.println(tuningData.checksum);

          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(data, sizeof(TuningData));
          Udp.endPacket();
          delay(200); // don't forget to delay
        }

        Serial.println();
      }
    }
  }
}

void Debug(void)
{
  Serial.print("Ref ");
  Serial.print(ref);
  Serial.print("     ");
  Serial.print("State ");
  Serial.print(state);
  Serial.print("     ");
  Serial.print("Error ");
  Serial.print(error);
  Serial.print("     ");
  Serial.print("Error_sum ");
  Serial.print(error_sum);
  Serial.print("     ");
  Serial.print("Error_dot ");
  Serial.print(error_dot);
  Serial.print("     ");
  Serial.print("Output ");
  Serial.println(output);
}

void PID(void)
{
  digitalWrite(LED, 0);
  /* if fix referent */

  prev_state = state ;
  state = prev_state + (filter * (((float)(analogRead(hall)) / 100.0f) - prev_state));

  prev_error = error;
  error = ref - state;
  error_dot = (error - prev_error) * 200.0f;
  error_sum = error_sum + error / 200.0f;

  output = kp * error + ki * error_sum + kd * error_dot + 220;

  if (output < 0)   output = 0;
  if (output > 1023) output = 1023;

  if (state < 6 || state > 9.5f)
  {
    output = 0;
    error_sum = 0;
    error_dot = 0;
    error = 0;
  }
  //  output = 0;
  analogWrite(magnetic, output);
  digitalWrite(LED, 1);
}
