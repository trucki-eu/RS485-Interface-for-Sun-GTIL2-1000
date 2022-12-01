# RS485-Interface-for-Sun-GTIL2-1000/2000
T3PL: Trucki's 3Phase limiter for SUN GTIL2-1000/2000 inverter:
---------------------------------------------------------------
UPDATE 28.11.2022: Adapter board to mount WEMOS modules on my pbc. To prevent stability issues with ESP8266 modules add a capacitor to my pcb:
https://github.com/trucki-eu/Trucki2Shelly-Gateway/blob/main/README.md#wemos-adapter

UPDATE 29.10.2022: Trucki2Shelly Gateway connects a Shelly 3EM to Trucki's RS485 interface pcb https://github.com/trucki-eu/Trucki2Shelly-Gateway

UPDATE 04.09.2022: Firmware 1.06 supports SDM630 Smartmeter 3Phase limiter support

![Overview T3PL](/assets/images/SDM630_mode_overview.PNG)

The new firmware 1.06 allows the RS485 interface pcb to communicate directly with the SDM630 smartmeter to limit the ac output power of the SUNG GTIL2 1000/2000 over three phases.

In case the firmware finds a SDM630 smartmeter on startup it uses the readings of the smartmeter to control the inverters ac output to reduce the house consuption to about 50W. 

The requirements for the SMD630 mode are:

The modbus ID on the RS485 interface pcb is set to ID:16, R19/J5 is close to use the RS485 port

![Overview SDM630Mode](/assets/images/SDM630mode.PNG)

The SDM630 smartmeter is set to Addr:001, 9k6, 8N1:

![Overview SDM630 Settings](/assets/images/SDM630_Settings.PNG)

You can check a working communication if you see the telefon icon on the SDM630 display. The SDM630 has to be powered before the inverter. On startup of the inverter you will the the telefon icon blink x1 while the RS485 interface pcb changes to the SDM630 mode. In SDM630 mode you will the the telefon icon to be permanent on:

![Overview SDM630 working communication](/assets/images/SDM630_successful_communication.PNG)

If you want to start the calibration mode of the RS485 interface pcb turn the inverter off, set the SDM630 to addr: 002 and turn the inverter on again. The inverter will ramp up the ac output. The calibration is done if the ac output of the inverter is back to 0/75W. If you are beginner you should work with the standard LUT.Calibration is fine tuning.

As the minimum controled output of the GTIL2-1000 is ~50W and for the GTIL2-2000 about 100W the SDM630 mode needs at least 50/100W house consumption to work properly.

Optional monitoring:

![Overview SDM630 ESPHome HomeAssistant monitor](/assets/images/SDM630_ESPHome_HomeAssistant_monitor.PNG)

In SDM630 mode the RS485 interface pcb uses the UART TX Pin to send a status telegram every 2s. As mentioned before the R19/J5 has to be closed for SDM630 mode anyway.
The status telegram follows the modbus protocol and contains six static start bytes: 0x10, 0x10, 0x00, 0x00, 0x00, 0x09, 0x12. The start bytes are followed by nine 16bit words of data: AC Display, AC Setpoint, VGrid, VBat, DAC Value, Calibration step, Inverter Temperature, SDM630 Power (4 byte float). The final 2 bytes is a 16bit CRC.

You can use ESPHome to send the status values to HomeAssistant, ioBroker, etc. Just connect +5V, GND and RX of an ESP8266/32 module to the UART port of the RS485 interface pcb:

![Overview SDM630 working communication](/assets/images/SDM630_optional_monitoring_esp8266.PNG)

The ESP Module can be flashed with ESPHome. You can find the matching gtil2sdm630.yaml/h file in the /code/esphome/sdm630 folder of the repository. Please make sure to copy the gtil2sdm630.h file into your /config/ESPHome folder in order to compile/install the GTIL2SDM630 project onto your ESP.

RS485 Modbus Interface for SUN GTIL2-1000/2000 MPPT inverter:
---------------------------------------------------------
UPDATE 18.08.2022: The RS485 Interface pcb now works for the SUN GTIL2-2000 version, too. 

![Overview](/assets/images/overview.PNG)

The GTIL2 RS485 Interface can be used to control the SUN GTIL2-1000/2000 MPPT inverter via Modbus RS485. The interface is connected between the inverter and the inverter display (4Pin JST2.54). The AC output of the inverter is controlled with a DAC[0..1.65V] via the analog input (RT1, 2Pin JST2.54). The RS485 interface is powered by the 12V power supply for the display. The DAC and the display communication line are electrical isolated from the RS485 interface and the microcontroller. The analog DAC output and the display line input are protected by 3.3V suppressor diodes. All limit modes/ internal / external limiters in the stettings menu of the SUN GTIL2-1000/2000 MPPT inverter have to be deactivated. The "Bat or soloar limited currect mode" has to be active with maximum current (35A @ 1000W).

However the usage of this design is still at your own risk. Please make sure that the upper pin of the inverters ac output is connected to LIVE and the lower pin to NEUTRAL. Otherwise you risk to get shocked. I.e. by a pin of the connector for the internal limiter. 

<img src="/assets/images/AC-Connection.jpg" width="500">

Mounting the RS485 interface pcb:
---------------------------------
![RS485 Interface](/assets/images/RS485_Interface.jpg)
Mouting the RS485 interface in the SUN GTIL2-1000/2000 MPPT is easy Plug'n Play. No soldering, no permanent modifications on the inverter. Just unscrew the 8x screws to open the inverter and disconect the the display cable on the inverter side. Mount two M4x10mm threaded standoff spacers in the prepared threads in the top area of the inverter. Mount the RS485 interface pcb with two spacers. Use a 1:1 4Pin 2.54mm JST cable to connect the inverter to the RS485 interface pcb and use the 2nd 4Pin connector on the RS485 interface pcb for the cable to the display. Use a 
1:1 2Pin 2.54mm JST cable to connect the RS485 interface DAC output (RT1) to the analog RT1 input of the inverter. Disconnect the cable for the external/internal limiter from the inverter and connect it to the 2 Pin RS485 port of the RS485 interface pcb.

<img src="/assets/images/Standoffs.jpg" height="400"> <img src="/assets/images/mounted-pcb.jpg" height="400"> <img src="/assets/images/mounted with cables.jpg" height="400">


Make sure to select the Modbus slave ID on the RS485 interface pcb with J1 - J4 before closing the case.

Modbus ID switch config: 
------------------------
| ID | J1 | J2 | J3 | J4 |
| -- | -- | -- | -- | -- |  
| 01 | OFF | OFF | OFF | OFF |
| 02 | ON  | OFF | OFF | OFF |
| 03 | OFF | ON  | OFF | OFF |
| 04 | ON  | ON  | OFF | OFF |
| 05 | OFF | OFF | ON  | OFF |
| 06 | ON  | OFF | ON  | OFF |
| 07 | OFF | ON  | ON  | OFF |
| 08 | ON  | ON  | ON  | OFF |
| 09 | OFF | OFF | OFF | ON  |
| 10 | ON  | OFF | OFF | ON  |
| 11 | OFF | ON  | OFF | ON  |
| 12 | ON  | ON  | OFF | ON  |
| 13 | OFF | OFF | ON  | ON  |
| 14 | ON  | OFF | ON  | ON  |
| 15 | OFF | ON  | ON  | ON  |
| 16 | ON  | ON  | ON  | ON  |
  
After you powered the inverter you should see a slowly (1s) blinking green LED. A fast blinking (100ms) LED means a problem with the display communication. 
  
1st Test with PC & RS485 USB converter stick:
---------------------------------------------
For a first test you can use a cheap USB RS485 converter stick from Ebay. Just connect Pin B to the Pin of the internal limit connector which is next to the DC input. Pin A is the pin of the internal limit connector which is more away from the inverter DC input. The corresponding connector on the cables side is called: GX12-female.

<img src="/assets/images/GTIL_with_USB_RS485_Interface.jpg" height="600">

As PC software you can use any Modbus software. I.e. Modbus VCL from http://www.ozm.cz/ivobauer/modlink/In Modbus VCL first set the Modbus Client/Slave ID(J1-J4) unter "Tools->Modbus Client Options". 

<img src="/assets/images/Modbus_VCL_Client Options.PNG" height="100">

Use "Tools->Modbus Connection Options->Serial Communication" and select the Serial Port of the RS485 USB converter. Baud Rate: 9600, Data Bits: 8 bits, Parity: None, Stop Bits: 1bit, Transmission Mode: RTU, Flow Control: None, Enabled Lines: DTR,RTS both unchecked, Silent Interval: 4. On the Register card: "Modbus Transaction Management -> Connection Mode:" Client, Refetch Delay [ms]: 0,Send Timeout [ms]: 1000, Receive Timout[ms]: 1000, Maximum Retries: 1, Turnaround Delay [ms]: 100, Thread Priority: Normal. 


<img src="/assets/images/MOdbus_VCL_Connection Options.PNG" height="300"> <img src="/assets/images/Modbus_VCL_Transaction_Management.PNG" height="300">

After selecting all settings go to the register card: "Register Access" and enter unter "Register Read": "Start Address: 0" , "Count: 6" and hit "Read input Registers" .In the log window you should now see:

<img src="/assets/images/Modbus_VCL_Read.PNG" height="400">
  
The available Modbus RTU commands / registers are:

Register functions:
| Reg| Description | Time |
| -- | ----------- | ---- |
| [0] | Set inverter AC output power [W*10] | (Update rate ~100ms)|
| [1] | Read display AC output power [W*10]| (Update rate ~1.3s) |
| [2] | Read display grid voltage [V*10]| (Update rate ~1.3s) |
| [3] | Read display battery [V*10]| (Update rate ~1.3s) |
| [4] | Set / Read DAC Value; [0]=0! | (Update rate ~100ms) |
| [5] | =1 Start Calibration. 17 Steps | (10s per step) |
| [6] | Mirror of REG[0] [W*10] | (Update rate ~100ms) Firmware >= 1.06|
| [7] | Temperature [0..118°C] | (Update rate ~100ms) Firmware >= 1.06|


You can use the "Register Write" section to manipulate the output of the SUN GTIL2-1000/2000 MPPT inverter. To set an output of 50W use "Start Address: 0", "Count: 1", "Value: 500" and press "Write Single/Multiple Register(s)" . If you press "Read Input Registers" again you will see that Register[0] = 500 (Set AC Output) and Register[4] = 577 (DAC Value). Register[1] shows the AC display output power. After each display read (every 1.3s) Register[4] (DAC Value) will be corrected to adjust Register[1] (AC Display Power) as close as possible to the setpoint of Register[0] (Set AC Output). Set Register[0]=0 to turn OFF the ac output.
The values of register[0-4] are multiplied with factor 10.
  
Calibration:
------------
In oder to get a fast (~100ms) AC output after a received modbus value on register[0] the RS485 interface pcb has two (1000/2000W) lookuptables (LUT) to link the AC setpoint to a DAC value. Both LUTs consits of 17 DAC values with related AC values. The conversion from a AC setpoint to a DAC value is done by interpolation between the 17 LUT entries. The standard LUT for the SUN GTIL2-1000 looks like this: 

| LUT No.   | 00 | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |  
| --------- | -- | -- | -- | -- | -- | -- | -- | -- | -- | -- | -- | -- | -- | -- | -- | -- | -- |
| AC [W*10] | 370 | 420 | 470 | 520 | 550 | 600 | 670 | 750 | 800 | 940 | 1100 | 1250 | 1850 | 1950 | 3500 | 3510 | 9600 |
| DAC | 279 | 400 | 514 | 620 | 680 | 775 | 895 | 1014 | 1079 | 1222 | 1315 | 1392 | 3282 | 3673 | 12180 | 12307 | 33187 |

Due to tolerances of the DAC, the 3.3 voltage regulator and the inverter itself the LUT is more or less accurate. As I only own the 1000W version, the LUT for the 2000W version might be wrong. To calibrate the LUT, write register[5]=1 and make sure that the DC supply of the inverter is strong enough to power the inverter at maximum output. The calibration will first initialize the active LUT with the standard LUT (Step 1). Starting with Step 2 the calibration will step the 17xDAC values of the LUT and save the matching AC values to the eeprom. Each step takes about 10s. The calibration is ready if register[5]=0 again.
 

2000W Version particularities:
------------------------------
The 2000W version is very similar to the GTIL2-1000 with some very minor particularities regarding the analog input connector RT1.
In general both versions allow to control the AC ouput via a 0..1.67V signal. For the 1000W version the AC output (0-1000W)  is quite linear to the analog signal (0-1.67V).
The 2000W version has a minimum AC output of 75W, which is related to 0V on the analog RT1 input. Means with the RS485 interface pcb an AC output with less than 75W is not possible. A stable control of the AC output is possible from ~100-2000W.

![LUT2000W](/assets/images/GTIL2_2000_RT1vsAC.PNG)

Furthermore the 2000W version does some kind of range switching from ~180W-250W.

![Hystereses2000W](/assets/images/Hysterese_GTIL2_2000W.PNG)

This means that AC outputs from 200W-250 are not possible and AC outputs from 180W-200W are only possible if the actual AC output is less than 200W.
The RS485 interface pcb firmware overwrites AC setpoints from 200W-225W -> 200W and 225W-250W ->250W. If the actual AC output is > 200W the firmware overwrites AC setpoints from 180-200W -> 180W.
For most applications these particularities are not disturbing. Using the RS485 interface pcb with a GTIL2-2000 and a 15S LiFePO4 battery I was able to a maximum stable AC output of ~2000W with the following settings:

![Settings2000W](/assets/images/Einstellungen_GTIL2_2000_150822.PNG) 

![Settings2000W](/assets/images/GTIL2_2000_AC_MAX.png)

!!! Do NOT run your Inverter on maximum output for a long time. It can overheat. The maximum AC output for the 1000W Version is 850W, for the 2000W version 1850W !!!

Standby:
--------
The 0W AC output standby power consuption of the SUN GTIL2-1000/2000 MPPT inverter in analog mode is about 10W on the DC side. You can reduce the standby current by setting the "Bat or solar limited current mode" voltage and the "Reboot voltage". I use 47V and 48V reboot voltage for a 15S LiFePo4 battery. With this  setting standby consuption stops as soon as the battery is below 47V. Sometimes the RS485 interface shows an AC display output of 7-9W, but the display shows 0W. I think the inverter sends 7-9W to the display, but the display shows very small AC outputs as 0W.

<img src="/assets/images/Display.jpg" height="400">

UART instead of RS485:
----------------------
If you want to use the UART port instead of the RS485 port just unsolder resistor R19.The UART port can be used for a direct/cross connection to a i.e. ESP8266. Please be aware that the UART port is working with 5V.  

Arduino:
--------
(A Zero export example with a SDM630 3-phase energy meter can be found here: https://github.com/trucki-eu/SDM630-zero-export-controller-Arduino- )

You can combine a SoftwareSerial lib with the ModbusMaster lib (by Doc Walker) to control the RS485 Interface pcb. Connect the pins D8/D9 of the master Arduino to the UART Port. If you power the Arduino master from another power supply (i.e. PC USB) do not connect 5V to your master Arduino:

<img src="/assets/images/Arduino.PNG" height="400">

Flash the following code to your master Arduino. The master Arduino reads all registers every second and prints the results to the serial console.
You can send the commands 'a', 'd' and 'c' to set an AC output Setpoint, to set a DAC value or to start a calibration. I.e. to set an AC output of 50W send 'a500'. To set a DAC value of 700 make sure that the ac output is zero and send 'd700'. To start a calibration make sure that ac output and dac value are zero and send 'c1'.

The screenshot shows the Arduino terminal with the actual registers and an ac output command for 50W.

<img src="/assets/images/arduino_terminal.PNG" height="400">

```
/*----------------------------------------------------------------------------------------------------
Modbus Master to test RS485 interface pcb for GTIL2 Sun 1000/2000 Inverter
This code is for a Modbus Master to test the RS485 GTIL2 interface PCB.
It is NOT the code for the RS485 interface PCB

Detailed description and screenshots: https://github.com/trucki-eu/RS485-Interface-for-Sun-GTIL2-1000#arduino

Uncomment "AltSoftSerial"  lines for ATMega328P boards and
          "SoftwareSerial" lines for ESP8266 boards

If you use a ESP8266 Board connect
ESP8266 Pin D1 -> TX Pin of the interface PCB
ESP8266 Pin D2 -> RX Pin of the interface PCB
ESP8266 GND    -> GND Pin of the interface PCB
If your ESP8266 Board is powered via USB do not connect 5V
Don't forget to remove R19 on the RS485 interface pcb if you use its UART port

 */
//------------------------------------------------------------------------------------------- 
#include <AltSoftSerial.h>                            //Use for Atmega328p
AltSoftSerial   mySerial(8, 9); // RX, TX             //Use for Atmega328p SoftwareSerial for communication with Sun GTIL2 interface

//#include <SoftwareSerial.h>                             //Use for ESP8266
//SoftwareSerial  mySerial(D1, D2); // RX, TX             //Use for ESP8266

#include "ModbusMaster.h" //https://github.com/4-20ma/ModbusMaster - by DocWalker
ModbusMaster    node;
int             id = 1;                               //Modbus RTU slave id of Sun GTIL2 interface
unsigned long   previousMillis_modbus = 0;            //Counter to next modbus poll
//-----------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);                                 //open serial port
  mySerial.begin(9600);                               //SoftwareSerial for communication with Sun GTIL2 interface
  node.begin(id, mySerial);                           //id=Modbus slave/client id of the interface pcb (Sun GTIL2)
  Serial.println("Start");   
}
//-----------------------------------------------------------------------------------------------------------------
void loop() { 
  if ( (millis()-previousMillis_modbus) >= 1000){      //Get new values from Sun GTIL2 every 1000ms
    uint8_t result = node.readInputRegisters(0, 6);    //Read register[0-5]
    uint16_t set_ac_power = node.getResponseBuffer(0);
    uint16_t ac_power     = node.getResponseBuffer(1);
    uint16_t vgrid        = node.getResponseBuffer(2);
    uint16_t vbat         = node.getResponseBuffer(3);
    uint16_t dac          = node.getResponseBuffer(4);
    uint16_t cal_step     = node.getResponseBuffer(5);   
    
    if (result == node.ku8MBSuccess) {                 //Serial print new values if received
      Serial.print("Set AC [W*10]: ") ;Serial.println(set_ac_power );
      Serial.print("AC     [W*10]: ") ;Serial.println(ac_power);   
      Serial.print("VGrid  [V*10]: ") ;Serial.println(vgrid);       
      Serial.print("VBat:  [V*10]: ") ;Serial.println(vbat);       
      Serial.print("DAC:           ") ;Serial.println(dac);
      Serial.print("CAL Step:      ") ;Serial.println(cal_step);
      Serial.println();
    } else Serial.println("Can not connect to Sun GTIL2 interface");     
    previousMillis_modbus = millis();  
  }
  
  if(Serial.available()){                              //Data received from PC
        String   received_str = Serial.readStringUntil('\n');
        String   my_float = received_str.substring(1);
        uint16_t my_value = my_float.toFloat();
        if(received_str[0] == 'a')                     //Send 'a500' to set ac output setpoint to 50W 
          if ( (my_value >=0) && (my_value <= 20000) ) //Check if output power is in range (0-2000W)
            node.writeSingleRegister(0,my_value);      //send new ac output setpoint via modbus rtu
        if(received_str[0] == 'd')                     //Send 'd500' to set DAC value to 500
          if ( (my_value >=0) && (my_value <= 33187) ) //Check if dac value is in range (0-33187W)
            node.writeSingleRegister(4,my_value);        
        if(received_str[0] == 'c')                     //Send 'c1' to start calibration; Send 'c99' for standard LUT
          if ( (my_value >=0) && (my_value <= 99) )
            node.writeSingleRegister(5,my_value);
   }
}
//-----------------------------------------------------------------------------------------------------------------
```

Raspberry:
----------
As the UART port of the Raspberry is NOT 5V tolerant you have to use a voltage divider on the Raspberry RXD line:

<img src="/assets/images/Raspberry.PNG" height="300">

MBPoll can be used as Modbus RTU Master on the Rasperry. To install MBPoll use the commands:
```
sudo apt update
sudo apt install mbpoll
```
Make sure that Rasperry's UART port is enabled:
```
sudo raspi-config

->Interfacing Options->P6 Serial
-Would you like a login shell to be accessible over serial? --> NO
-Would you like the serial port hardware to be enabled? --> YES
-Finish raspi-config, yes, reboot!
```
To read values use the command. If /dev/ttyAMA0 doesn't work try /dev/ttyS0 instead
```
mbpoll -a 01 -b 9600 -t 3 -r 1 -c 6 /dev/ttyAMA0  
```
The console output should look like this:

<img src="/assets/images/mbpoll_read.PNG" height="300">

To set the inverter output to 50W use the following command. If /dev/ttyS0 doesn't work try /dev/ttyAMA0 instead
```
mbpoll -a 01 -b 9600 -t4:int -r 1 /dev/ttyS0 -- 500
```
The console output should look like this:

<img src="/assets/images/mbpoll_write.PNG" height="300">


Controlling the SUN GTIL2 from HomeAssistant:
---------------------------------------------
Connect a ESP8266 Module to the UART Port and use i.e. HomeAssistant to communicate with the Sun GTIL2 inverter.
The ESP8266 module has to be 5V tolerant and is directly powered from the RS485 interface pcb. 

<img src="/assets/images/ESP8266.PNG" height="300">

In HomeAssistant use ESPHome to communicate with the ESP8266 module. Make sure that you are on the lastest version of ESPHome. Create a new device in ESPHome and add the folowing lines to the configuration of the new device:
```
web_server:
  port: 80
  
# Enable logging
logger:
    level: DEBUG
    baud_rate: 115200
    hardware_uart: UART1 
    
uart:
  id: mod_bus
  tx_pin: 1
  rx_pin: 3
  baud_rate: 9600
  stop_bits: 1
  
modbus:
  #flow_control_pin: 5
  id: modbus1  
  
modbus_controller:
  - id: sun
    ## the Modbus device addr
    address: 0x1
    modbus_id: modbus1
    update_interval: 1s
    setup_priority: -10
    
sensor:
  - platform: modbus_controller
    modbus_controller_id: sun
    name: "AC Output"
    id: ac_output
    register_type: holding
    address: 0x01
    unit_of_measurement: "W"
    value_type: U_WORD 
    accuracy_decimals: 1
    filters:
      - multiply: 0.1      
  - platform: modbus_controller
    modbus_controller_id: sun
    name: "Grid Voltage"
    id: grid_voltage
    register_type: holding
    address: 0x02
    unit_of_measurement: "V"
    value_type: U_WORD
    accuracy_decimals: 1
    filters:
      - multiply: 0.1      
  - platform: modbus_controller
    modbus_controller_id: sun
    name: "Bat Voltage"
    id: bat_voltage
    register_type: holding
    address: 0x03
    unit_of_measurement: "V"
    value_type: U_WORD
    accuracy_decimals: 1
    filters:
      - multiply: 0.1
#  - platform: modbus_controller
#    modbus_controller_id: sun
#    name: "Temperature"
#    id: temperature
#    register_type: holding
#    address: 0x07
#    unit_of_measurement: "°C"
#    value_type: U_WORD

number:
  - platform: modbus_controller
    modbus_controller_id: sun
    id: ac_setpoint_number
    name: "AC Setpoint Number"
    address: 0x00
    value_type: U_WORD
    multiply: 10    
    unit_of_measurement: "W"
    min_value: 0
    max_value: 9600
  - platform: modbus_controller
    modbus_controller_id: sun
    id: dac_number
    name: "DAC Number"
    address: 0x04
    value_type: U_WORD
    min_value: 0
    max_value: 33187      
  - platform: modbus_controller
    modbus_controller_id: sun
    id: calibration_number
    name: "Calibration Number"
    address: 0x05
    value_type: U_WORD
    min_value: 0
    max_value: 1
```
After you have flashed the ESPHome configuration to your device you can create a new card in HomeAssistant which can look like this:

<img src="/assets/images/HomeAssistant.png" height="300">

Controlling the SUN GTIL2 with Tasmota:
---------------------------------------
Tasmota recently got a [*Modbus Bridge*](https://github.com/tasmota/docs/blob/development/docs/Modbus-Bridge.md) which we can use to communicate with the RS485 interface pcb. For ESP8266 Tasmota needs to be compiled with Modbus bridge support. You can download a ESP8266 Tasmota firmware with Modbus support [*here*](https://github.com/trucki-eu/RS485-Interface-for-Sun-GTIL2-1000/tree/main/code/tasmota/).

Once you have flashed the Tasmota firmware with modbus support to your ESP8266 you can log into the WebInterface to configure and test the modbus communication with the RS485 interface pcb.

At first go to the Tastmota configuration tab and set GPIO1 = ModBr Tx und GPIO3 = ModBr Rx :

![Tasmota Configuration](/assets/images/Tastmota_Configuration.PNG)

After the configuration connect your ESP8266 module to the RS485 interface pcb the same way as descibed in the HomeAssistant section of this page.

Then select "Consoles" in your Tasmota Webinterface and enter the following commands to configure the modbus settings:
```
ModbusBaudrate 9600
ModbusSerialConfig 8N1
```
To set an AC Output (REG[0]) of 100W (value=1000) enter the following command in the console command input:
```
MODBUSSEND {"deviceaddress": 1, "functioncode": 6, "startaddress": 0, "type":"uint16", "count":1, "values":[1000]}
```

To read all Registers from the RS485 interface pcb use the following command. Use count:6 for firmware <1.06 and count:8 for firmware >=1.06
```
MODBUSSEND {"deviceaddress": 1, "functioncode": 3, "startaddress": 0, "type":"uint16", "count":6}
```
You will get the following consoles output:
![Tasmota Configuration](/assets/images/Tasmota_Modbus_Consoles.png)

Updates:
--------
1) Download AVRDude.exe,conf,pdb from https://github.com/mariusgreuel/avrdude/releases
2) Connect a USB TTL adapter to the RS485 Interface pcb.

USB -> RS485 Interface pcb

5V  -> 5V

TXD -> RXD

RXD -> TXD

GND -> GND

![USB TTL Adapter to update firmware](/assets/images/update_firmware_hardware.png) 

3) Remove R19 (or J5) to use UART Port. RS485 interface pcb blinks fast green if powerd via USB.
4) Create a new Batch file with the following content. Adapt the COM-Port to the port of your USB TTL adapter. Place all files (AVRDude exe,pdb,conf , *.hex, *.bat) a folder.
```
avrdude -Cavrdude.conf -v -patmega328p -carduino -PCOM9 -b57600 -D -Uflash:w:GTIL_RS485_DAC_Display_V1.04.ino.hex:i 
pause
```
5) Start batch file.  AVRDude waits for manual rest of the RS485 interface pcb

![AVRDude waiting for manual reset](/assets/images/AVRDude_waiting_for_manual_reset.PNG)

6) Connect RST & GND on the RS485 Interace pcb shortly until AVRDude starts to flash and the green led turns off. This part might be tricky. You might have to reset the board several times. If you reset the board during the flash process you will not brick it, but you have to flash it again.

7) If flashing was successful you will see this message and the green led starts blinking again.

![flash successful](/assets/images/update_flash_successful.PNG)
