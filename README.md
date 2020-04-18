# VL530l0X_ESP8266
TOF Sensor


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
	  
