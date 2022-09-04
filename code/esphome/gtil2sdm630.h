#include "esphome.h"

uint8_t data[27] = {0x10, 0x10, 0x00, 0x00, 0x00, 0x09, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF}; //8E 00
int data_cnt = 0;
uint16_t u16CRC = 0xFFFF;
//------------------------------------------------------------------------------
class gtil2sdm630 : public PollingComponent, public UARTDevice{
public:
    Sensor* ac_setpoint = new Sensor(); //AC setpoint [W*10]
    Sensor* ac_display  = new Sensor(); //AC display  [W*10]
    Sensor* vgrid       = new Sensor(); //VGrid       [V*10]
    Sensor* vbat        = new Sensor(); //VBat        [V*10]
    Sensor* dac         = new Sensor(); //DAC         [0..32767]
    Sensor* calibration = new Sensor(); //Calibration [0..99]
    Sensor* temperature = new Sensor(); //Temperature [0..118°C]
    Sensor* sdm630_pa   = new Sensor(); //SDM630 PA   [W]

	gtil2sdm630(UARTComponent* parent) : PollingComponent(1000), UARTDevice(parent) {}

//------------------------------------------------------------------------------
static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; ++i) if (crc & 1) crc = (crc >> 1) ^ 0xA001; else crc = (crc >> 1);
  return crc;
}
//------------------------------------------------------------------------------
float reform_uint16_2_float32(uint16_t u1, uint16_t u2){
  //convert SDM630 2xuint16_t to float
  uint32_t num = ((uint32_t)u1 & 0xFFFF) << 16 | ((uint32_t)u2 & 0xFFFF);
  float numf;
  memcpy(&numf, &num, 4);
  return numf;
} 
//------------------------------------------------------------------------------
	void sensor(Sensor* sensor, float value)
	{
		if (sensor->state != value)
		{
			sensor->publish_state(value);
		}
	}
//------------------------------------------------------------------------------
	void setup() override
	{
        flush();
	}
//------------------------------------------------------------------------------
	void update() override
	{
       while(available()){
         char c = read();
         switch (data_cnt) {
           //byte 0-6: check for delimter 0x10,0x10,0x00,0x00,0x00,0x09,0x12
           case  0       : u16CRC = 0xFFFF;
           case  1 ... 6 : if(c == data[data_cnt]){                              
                             u16CRC=crc16_update(u16CRC,data[data_cnt++]);
                           }else data_cnt=0; 
                           break;  
           //save byte 7-24 to data
           case  7 ... 24: data[data_cnt]=c;                                      
                           u16CRC=crc16_update(u16CRC,data[data_cnt++]);
                           break;                                                                               
           //crc
           case 25       : data[data_cnt++]=c; break;                  
           case 26       : data[data_cnt]=c;                                      
                           if((data[data_cnt-1]==lowByte(u16CRC))&&(data[data_cnt]==highByte(u16CRC)) )data_cnt++;
                           else data_cnt=0;
                           break;                                                                                                 
         }
         if (data_cnt == sizeof(data)){ 
           sensor(ac_setpoint, data[7]*256 +data[8]);
		   sensor(ac_display,  data[9]*256 +data[10]);
		   sensor(vgrid,       data[11]*256+data[12]);
		   sensor(vbat,        data[13]*256+data[14]);
		   sensor(dac,         data[15]*256+data[16]);
		   sensor(calibration, data[17]*256+data[18]);
		   sensor(temperature, data[19]*256+data[20]);
		   sensor(sdm630_pa,   reform_uint16_2_float32(data[23]*256+data[24],data[21]*256+data[22]));
           data_cnt = 0;
         }  
       } 
	}
//------------------------------------------------------------------------------
};
