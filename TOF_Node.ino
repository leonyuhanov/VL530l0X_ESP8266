/*
      Coded for the Wemos D1 Mini (in board manager select)
      - Lolin(WEMOS) D1 R2 & Mini
      - CPU Freq: 160Mhz
      - Erase Flash: All Flash Contents

      Requires the Pololu VL530l0X Lib
      https://github.com/pololu/vl53l0x-arduino
      
      COnect VL530l0X pins as follows:
      -VCC -> ESP8266 3.3V
      -GND -> ESP8266 GND
      -SCK(CLK) -> ESP8266 D1
      -SDA(DAT) -> ESP8266 D2
      -XSHUT Not connected
      -SPIO1 Not connected
	  
	   Sensors I2C Address is hard coded to 41 (0x29)
	  
 */

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X TOFSenseModule;
unsigned long delayBetweenRangeReading = 100;
unsigned short int readValue=0;
const byte numerOfSamples = 5;
float rangeSamples[numerOfSamples];
unsigned short int sampleCounter=0;
byte percentileScaledValue=0;
//{Closest VALID distance from sensor, Furthest VALID distance from sensor,0,0}
float readMargins[4] = {20, 1000, 0, 0};
float previousRange=0, currentRange=0, currentSlope=0, rangeAverage=0, previousRangeAverage=0;
float currentVelocity = 0, currentVelocityinMMPerSecond=0;
float rangeAverageFlutter = 3;
unsigned long rangeTimers[3] = {0,0,0}, txTimers[3] = {0,0,0};
//Clock and DATA pins for i2c buss
byte I2C_CLOCK_PIN = D3;
byte I2C_DATA_PIN = D4;
//Network Stuff
//Please enter corect details for wifi details
const char * ssid = "Studio";
const char * password = "fa5fa5fa55";
WiFiUDP Udp;
const unsigned int udpTXPort = 1000;
//Each sensor node needs its own unique ID
const byte nodeID = 5;
/*
Node Type 
0=TOF 
1=BUTTON (Not yet impleneted but compatible)
*/
const byte nodeType = 0;
//Data packet that is sent when a valid ranging is performed
/*Byte  Value
  0     NodeID
  1		  Node Type: 0=TOF 1=BUTTON
  2     Percentile Range based on readMargins from 0 to 100
  3     Float[i0]:  Actual Range Reading
  4     Float[i1]:  Actual Range Reading
  5     Float[i2]:  Actual Range Reading
  6     Float[i3]:  Actual Range Reading
  7     Float[i0]:  Velocity
  8     Float[i1]:  Velocity
  9     Float[i2]:  Velocity
  10     Float[i3]:  Velocity
  11    Float[i0]:  Avg Range Reading
  12    Float[i1]:  Avg Range Reading
  13    Float[i2]:  Avg Range Reading
  14    Float[i3]:  Avg Range Reading
 */
const byte packetSize=15;
byte dataToSend[packetSize];
//Before uploading this code to each sensor node, set up the server node and obtanin its IP address and enter it here
//Im assuimng the IP address of the server is 192.168.1.250
IPAddress serverAddress(192,168,1,250);

void setup() 
{
  //Enable Debug to Serial Port 1
  Serial.begin(115200);
  Serial.printf("\r\n");
  //Eable WIFI
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
        delay(100);
        Serial.print(".");
  }
  if(nodeType==0)
  {
    Serial.printf("\r\nTOF Sense Module\t%d\tis ONLINE\r\nCurrent Ip Address:\t", nodeID);
  }
  else if(nodeType==1)
  {
    Serial.printf("\r\nBUTTON Sense Module\t%d\tis ONLINE\r\nCurrent Ip Address:\t", nodeID);
  }
  Serial.print(WiFi.localIP());
  Serial.print("\r\n");
  
  if(nodeType==0)
  {
  //Init I2C
    Wire.begin(I2C_DATA_PIN, I2C_CLOCK_PIN);
    //Init TOF Module
    TOFSenseModule.init();
    TOFSenseModule.startContinuous();
    //set up sense margins
    readMargins[2] = readMargins[1]-readMargins[0];
  }
  else if(nodeType==1)
  {
    //Init Buttons
  }
  //Set this Nodes ID
  dataToSend[0] = nodeID;
  dataToSend[1] = nodeType;
  startTimer(1, txTimers);
}

void loop() 
{
  rangeTimers[0] = millis();
  getRange(&currentRange);
  while(currentRange==-1)
  {
    rangeTimers[0] = millis();
    getRange(&currentRange);
    yield();
  }
  currentSlope = getSlope(previousRange, currentRange);
  rangeTimers[1] = millis();
  rangeTimers[2] = rangeTimers[1]-rangeTimers[0];
  //Set percentile range value based ont he read margins
  percentileScaledValue = ((currentRange-readMargins[0])/readMargins[2])*100;
  //Run averaging to work out if ts still or not
  rangeAverage = getAvgRange(&currentRange, rangeSamples, &sampleCounter);
  if(rangeAverage > (previousRangeAverage+rangeAverageFlutter) || rangeAverage < (previousRangeAverage-rangeAverageFlutter))
  {
    if(currentSlope>0)
    {
      //Movement AWAY from the TOF module
      currentVelocity = ((currentRange - previousRange)/1000) / ((float)rangeTimers[2]/1000);
    }
    else if(currentSlope<0)
    {
      //Movement TOWARDS to the TOF module
      currentVelocity = ((previousRange - currentRange)/1000) / ((float)rangeTimers[2]/1000);
    }
    else
    {
      currentVelocity = 0;
    }
  }
  else
  {
    //STILL
    currentVelocity = 0;
  }

  //calculate Velocity in M/s 
  currentVelocityinMMPerSecond = (currentRange - previousRange) / ((float)rangeTimers[2]/100);
  //Output to console
  Serial.printf("\r\n%f\t%f\t%f\t%d\t%f", currentRange, currentVelocityinMMPerSecond, rangeAverage, percentileScaledValue, currentVelocity);  
  
  previousRange = currentRange;
  previousRangeAverage = rangeAverage;

  //TX data every delayBetweenRangeReading 
  if( hasTimedOut(txTimers) )
  {
    txData();
    startTimer(delayBetweenRangeReading, txTimers);
  }
  delay(10);
}

void txData()
{
  byte cRange_ByteArray[3], cVelocity_ByteArray[3], avgRange_ByteArray[3];
  byte bIndex = 0;
  
  //convert currentRange (float) into byte array
  floatToByteArray(currentRange, cRange_ByteArray);
  //convert currentVelocityinMMPerSecond(float) into byte array
  //floatToByteArray(currentVelocityinMMPerSecond, cVelocity_ByteArray);
  //convert currentVelocity(float) into byte array
  floatToByteArray(currentVelocity, cVelocity_ByteArray);
  //convert readValue(float) into byte array
  floatToByteArray(rangeAverage, avgRange_ByteArray);
  //store byte arrays into the data packet to TX
  dataToSend[2] = percentileScaledValue;
  memcpy(dataToSend+3, cRange_ByteArray, 4);
  memcpy(dataToSend+7, cVelocity_ByteArray, 4);
  memcpy(dataToSend+11, avgRange_ByteArray, 4);
  
  //TX Packet
  Udp.beginPacket(serverAddress, udpTXPort);
  Udp.write(dataToSend, packetSize);
  Udp.endPacket();
}

void floatToByteArray(float fValue, byte* byteArray)
{
  byte* bytePointerToFloatValue = (byte*)&fValue;
  byteArray[0] = bytePointerToFloatValue[0];
  byteArray[1] = bytePointerToFloatValue[1];
  byteArray[2] = bytePointerToFloatValue[2];
  byteArray[3] = bytePointerToFloatValue[3];
}

void getRange(float *returnRange)
{
  readValue = TOFSenseModule.readRangeContinuousMillimeters();
  if(readValue>readMargins[0] && readValue<readMargins[1])
  {
    *returnRange = readValue;
  }
  else
  {
    *returnRange = -1;
  }
}

float getAvgRange(float *rangeRead, float *samples, unsigned short int *sCounter)
{
  byte isCnt=0;
  float avgRange=0;
  
  samples[*sCounter%numerOfSamples] = *rangeRead;
  if(*sCounter>numerOfSamples)
  {
    for(isCnt=0; isCnt<numerOfSamples; isCnt++)
    {
      avgRange += samples[isCnt];
    }
    avgRange = avgRange/numerOfSamples;
    *sCounter = *sCounter + 1;
    return avgRange;
  }
  *sCounter = *sCounter + 1;
  return avgRange;
}

float getSlope(float x1, float x2)
{
  float slope=0;
  
  if(x1!=x2)
  {
    slope = 1 / (x2-x1);
  }
  return slope;
}

byte hasTimedOut(unsigned long* timerArray)
{
  timerArray[1] = millis();
  if(timerArray[2] < timerArray[1]-timerArray[0])
  {
    return 1;
  }
  return 0;
}
void startTimer(unsigned long durationInMillis, unsigned long* timerArray)
{
  timerArray[0] = millis(); 
  timerArray[2] = durationInMillis;
}
