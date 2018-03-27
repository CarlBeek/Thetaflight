/*
Thetaflight for FlopBot
By Carl Beekhuizen


Wiring:
Assuming an Arduino Micro:

0			(RX)		CPPM
2			(SDL)		I2C
3			(SCL)		I2C
7			(DRDY)		HMC DRDY Interrupt
9			(PWM)		Servo
10			(PWM)		Motor
13			(BIN)		LED

Channels:
RX 			Arduino		Name
1			0			Aileron
2			1			Elevator
3			2			Collective
4			3			Rudder
5			4			Motor Slider (Throttle)
6			5			Arm Switch
7			6			-
8			7			-
9			8			Frame Buffer

Arduino Micro Timers:
Timer 		bits 		Prescaling	Type		TOP		 	Frequency	Useage
0			8			64			CTC			77			3205		LSM6 && ADXL
1A			16			256			Fast PWM	999			250			Servo PDM
1B			16			256			Fast PWM	999			250			Motor PDM
*/



//Inclusion of libraries
#include <Wire.h>

//I2C Addresses
#define HMC_ADDRESS						(0x1E)

//Servo definitions
#define servo_centre 					(1500)
#define servo_max_lift					(1000)
#define servo_min_lift					(2000)
#define motor_min						(1000)

//Definitions for HMC5883L
#define HMC_REG_MAG_CRA_REG_M			(0x00)			//ConfigA register
#define HMC_REG_MAG_CRB_REG_M			(0x01)			//ConfigB register
#define HMC_REG_MAG_MR_REG_M			(0x02)			//Mode register
#define HMC_REG_MAG_OUT_X_H_M			(0x03)			//High bits of X axis (First Axis)
#define HMC_REG_MAG_OUT_Z_H_M 			(0x05)			//High bits of Z axis (Axis about which compass is usefull)
#define HMC_GAUSS_LSB_XY				(1100)			//LSB to Gauss in XY
#define HMC_GAUSS_LSB_Z					(980)			//LSB to Gauss in Z

//sundry definitions
#define float_max 						(3.4028235E+38)
#define SUM_min							(243)

const float two_pi = 6.2831853;


volatile unsigned int cppm_channels[9] = {2000, 2000, 2000, 1000, 1000, 2000, 1000, 1000, 1000};
volatile boolean hmc_data_ready = 0;
unsigned int pwm_out[2];
//HMC Variables:
uint8_t hmc_buffer[6];
int16_t hmc_raw[3];
//Yaw and heading variables:
float heading_target = 0.0;
float heading_target_previous = 0.0;

void setup(){
	attachInterrupt(digitalPinToInterrupt(7), hmc_interrupt, FALLING);
	timer_interrupt_setup();
	hmc_setup();
	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);
	pinMode(12, OUTPUT);
	Serial.begin(9600);
	/*
	hmc_read();
	heading_target_previous = atan2(hmc_raw[2], hmc_raw[0]);
	*/
}

void loop(){
	//if there is new compass data
	if(hmc_data_ready){
		hmc_data_ready = false;
		hmc_read();
	}

	//Asimuth approximation:
	float mag_heading_current = atan2(hmc_raw[2], hmc_raw[0]);
	//Asuming 1000us looptime, this should result in 100dps max yaw
	heading_target = heading_target_previous + fap(1500,1000,2000,-0.6283185,0.6283185);
	heading_target_previous = heading_target;
	float heading_relative = relative_heading_calc(mag_heading_current, heading_target);

	// Serial.println(heading_relative);

	int component_pitch = sin(heading_relative)*(cppm_channels[1] - 1500);
	int component_roll = cos(heading_relative)*(1500 - cppm_channels[0]);
	int component_collective = 1500 + 0.5*(cppm_channels[2] - 1500);
	//Checks if system is armed with cppm_channel[5]
	if (cppm_channels[5]>1500){
		//turn on LED for rear 60 degrees
		digitalWrite(12, ((heading_relative>2.62) && (heading_relative<3.67)));
		pwm_out[0] = (((component_collective + component_roll + component_pitch) - SUM_min) / 2.9) + servo_max_lift;
		pwm_out[1] = 1000;
		
	}else{
		pwm_out[0] = 1500;
		pwm_out[1] = 1000;
		digitalWrite(12, LOW);
	}
	update_outputs();
}


float relative_heading_calc(float current, float target){
	int x = (int)((current + target)/ two_pi);
	float mod = ((current + target) - (x * two_pi));
	if(mod<0){
		return mod+two_pi;
	}else{
		return mod;
	}
}

//float Mapping function
float fap(int x, int in_min, int in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void timer_interrupt_setup(){
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;

	TCNT1H = 0;
	TCNT1L = 0;

	//Set TOP values. Counter runs from 0 - 999;
	ICR1H = highByte(999);
	ICR1L = lowByte(999);


	//Set OC1 A/B at TOP clear on match && WGM for Fast PWM ICR1
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	//Remaing WGM bits and a prescalar of 64
	TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
}

//configure compass
void hmc_setup(){
	// Register 0x00: CONFIG_A
  	// normal measurement mode (0x00) and 75 Hz ODR (0x18)
  	Wire.beginTransmission(HMC_ADDRESS);
  	Wire.write(HMC_REG_MAG_CRA_REG_M);
  	Wire.write((byte) 0x18);
  	Wire.endTransmission();
  	delayMicroseconds(5000);
 
	// Register 0x01: CONFIG_B
	// default range of +/- 130 uT (0x20)
  	Wire.beginTransmission(HMC_ADDRESS);
  	Wire.write(HMC_REG_MAG_CRB_REG_M);
  	Wire.write((byte) 0x20);
  	Wire.endTransmission();
  	delayMicroseconds(5000);
 
 	// Register 0x02: MODE
	// continuous measurement mode at configured ODR (0x00)
	// possible to achieve 160 Hz by using single measurement mode (0x01) and DRDY
	Wire.beginTransmission(HMC_ADDRESS);
	Wire.write(HMC_REG_MAG_MR_REG_M);
	Wire.write(0x01);
	Wire.endTransmission();
}



void hmc_interrupt(){
	hmc_data_ready = true;
}

//writes the pwm signals to the motor and servo
void update_outputs(){
	int servo_out = map(pwm_out[0], 1000, 2000, 250, 500);
	int motor_out = map(pwm_out[1], 1000, 2000, 250, 500);
	noInterrupts();
	OCR1AH = highByte(servo_out);
	OCR1AL = lowByte(servo_out);
	OCR1BH = highByte(motor_out);
	OCR1BL = lowByte(motor_out);
	interrupts();
}

// read 6 bytes (x,y,z magnetic field measurements) from the magnetometer
void hmc_read() {
	// multibyte burst read of data registers (from 0x03 to 0x08)
	Wire.beginTransmission(HMC_ADDRESS);
	Wire.write((byte) HMC_REG_MAG_OUT_X_H_M); // the address of the first data byte
	Wire.endTransmission();

	Wire.beginTransmission(HMC_ADDRESS);
	Wire.requestFrom(HMC_ADDRESS, 6);  // Request 6 bytes

	for(int i = 0; Wire.available() >0 && i < 6; i++){
		hmc_buffer[i] = Wire.read();  // Read one byte
	}
	Wire.read();
	Wire.endTransmission();

	// combine the raw data into full integers (HMC588L sends MSB first)
	//           ________ MSB _______   _____ LSB ____
	hmc_raw[0] = (hmc_buffer[0] << 8) | hmc_buffer[1]; //x
	hmc_raw[1] = (hmc_buffer[2] << 8) | hmc_buffer[3]; //z
	hmc_raw[2] = (hmc_buffer[4] << 8) | hmc_buffer[5]; //y

	// put the device back into single measurement mode
	Wire.beginTransmission(HMC_ADDRESS);
	Wire.write((byte) HMC_REG_MAG_MR_REG_M);
	Wire.write((byte) HMC_REG_MAG_CRB_REG_M);
	Wire.endTransmission();
}