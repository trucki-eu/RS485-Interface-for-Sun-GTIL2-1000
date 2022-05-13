# RS485-Interface-for-Sun-GTIL2-1000
RS485 Modbus Interface for SUN GTIL2-1000 MPPT inverter:
---------------------------------------------------------
![RS485 Interface](/assets/images/RS485_Interface.jpg)
The GTIL2 RS485 Interface can be used to control the SUN GTIL2-1000 MPPT inverter via Modbus RS485. The interface is connected between the inverter and the inverter display (4Pin JST2.54). The AC output of the inverter is controlled with a DAC[0..1.65V] via the analog input (RT1, 2Pin JST2.54). The RS485 interface is powered by the 12V power supply for the display. The DAC and the display communication line are electrical isolated from the RS485 interface and the microcontroller. The analog DAC output and the display line input are protected by 3.3V suppressor diodes. All limit modes/ internal / external limiters in the stettings menu of the SUN GTIL2-1000 MPPT inverter have to be deactivated. The "Bat or soloar limited currect mode" has to be active with maximum current (35A @ 1000W).

However the usage of this design is still at your own risk. Please make sure that the upper pin of the inverters ac output is connected to LIVE and the lower pin to NEUTRAL. Otherwise you risk to get shocked. I.e. by a pin of the connector for the internal limiter. 

<img src="/assets/images/AC-Connection.jpg" width="500">

Mounting the RS485 interface pcb:
---------------------------------
Mouting the RS485 interface in the SUN GTIL2-1000 MPPT is easy. Just unscrew the 8x screws to open the inverter and disconect the the display cable on the inverter side. Mount two M4x10mm threaded standoff spacers in the prepared threads in the top area of the inverter. Mount the RS485 interface pcb with two spacers. Use a 1:1 4Pin 2.54mm JST cable to connect the inverter to the RS485 interface pcb and use the 2nd 4Pin connector on the RS485 interface pcb for the cable to the display. Use a 
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
| [0] | Set inverter AC output power | (Update rate ~100ms)|
| [1] | Read display AC output power | (Update rate ~1.3s) |
| [2] | Read display grid voltage | (Update rate ~1.3s) |
| [3] | Read display battery | (Update rate ~1.3s) |
| [4] | Set / Read DAC Value; [0]=0! | (Update rate ~100ms) |
| [5] | =1 Start Calibration. 17 Steps | (10s per step) |

You can use the "Register Write" section to manipulate the output of the SUN GTIL2-1000 MPPT inverter. To set an output of 50W use "Start Address: 0", "Count: 1", "Value: 500" and press "Write Single/Multiple Register(s)" . If you press "Read Input Registers" again you will see that Register[0] = 500 (Set AC Output) and Register[4] = 577 (DAC Value). Register[1] shows the AC display output power. After each display read (every 1.3s) Register[4] (DAC Value) will be corrected to adjust Register[1] (AC Display Power) as close as possible to the setpoint of Register[0] (Set AC Output). Set Register[0]=0 to turn OFF the ac output.
  
Calibration:
------------
In oder to get a fast (~100ms) AC output after a received modbus value on register[0] the RS485 interface pcb has two (1000/2000W) lookuptables (LUT) to link the AC setpoint to a DAC value. Both LUTs consits of 17 DAC values with related AC values. The conversion from a AC setpoint to a DAC value is done by interpolation between the 17 LUT entries. Due to tolerances of the DAC, the 3.3 voltage regulator and the inverter itself the LUT is more or less accurate. As I only own the 1000W version, the LUT for the 2000W version might be wrong. To calibrate the LUT, write register[5]=1 and make sure that the DC supply of the inverter is strong enough to power the inverter at maximum output. The calibration will step the 17xDAC values of the LUT and save the matching AC values to the eeprom. Each step takes about 10s. The calibration is ready if register[5]=0 again.
  
Standby:
--------
The 0W AC output standby power consuption of the SUN GTIL2-1000 MPPT inverter in analog mode is about 10W on the DC side. You can reduce the standby current by setting the "Bat or solar limited current mode" voltage and the "Reboot voltage". I use 47V and 48V reboot voltage for a 15S LiFePo4 battery. With this  setting standby consuption stops as soon as the battery is below 47V. Sometimes the RS485 interface showsa AC display output of 7-9W, but the display shows 0W. I think the inverter sends 7-9W to the display, but the display shows very small AC outputs as 0W.

<img src="/assets/images/Display.jpg" height="400">
  
Arduino:
--------
If you want to use an Arduino as Modbus Master to control the RS485 Interface pcb you can use the following lines:
```
//First include the ModbusMaster libary:
#include "ModbusMaster.h" //https://github.com/4-20ma/ModbusMaster

void setup() {
  Serial.begin(9600);                                 //open serial port
  node.begin(id, Serial);                             //id=Modbus slave/client id of the RS485 interface pcb (Sun GTIL2)
  node.writeSingleRegister(0,500);                    //Write 50W AC outputpower to register[0]=500    

  node.readInputRegisters(0, 6);                      //Read register[0-5]
  uint16_t set_ac_power = node.getResponseBuffer(0);
  uint16_t ac_power     = node.getResponseBuffer(1);
  uint16_t vgrid        = node.getResponseBuffer(2);
  uint16_t vbat         = node.getResponseBuffer(3);
  uint16_t dac          = node.getResponseBuffer(4);
  uint16_t cal_step     = node.getResponseBuffer(5);
}

void loop() {
}
```

UART instead of RS485
---------------------
If you want to use the UART port instead of the RS485 port just unsolder resistor R19.The UART port can be used for a direct/cross connection to a i.e. ESP8266. Please be aware that the UART port is working with 5V.
  
Updates:
--------
If you want to update the firmware on the ATm3ega328 on the RS485 interface pcb you can use the Arduinio ide.
Just open the GTIL_RS485_DAC_Display_V1.00.ino and select
Tools->Board:     "Arduino Pro or Pro Mini"
Tools->Prozessor: "ATmega328P (3.3V,8MHz)"
  
If you connect a ISP Programmer to the ISP Port of the RS485 interface pcb you can use
Sketch->Upload with Programmer
  
If you want to upload the firmware via the UART port:

1)Remove R19 to use UART Port

2)Connect GND,RX,TX,5V to a USB->TTL Adapter (i.e. CH340)

3)Select Tools->Port: Com-Port of USB->TTL Adapter 

4)Connect RST&GND Pin

5)Press Sketch->Upload

6)Release RST&GND Pin as soon as you see "Uploading..."
  
