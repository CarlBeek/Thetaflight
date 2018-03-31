/*
Thetaflight for FlopBot
By Carl Beekhuizen


Version 1.01
-Flopbot v2
-Translational adaptation of working V0.03

Wiring:
Assuming an Arduino Micro:

0			(RX)		CPPM
2			(SDL)		I2C
3			(SCL)		I2C
7			(DRDY)		HMC DRDY Interrupt
9			(PWM)		Servo
10			(PWM)		Motor
12			(BIN)		LED

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
#define servo_max_lift					(2000)
#define servo_min_lift					(1000)
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

//Arduino pin definitions
#define servo_pin 9
#define motor_pin 10
#define heading_led_pin 12

//const float two_pi = 6.2831853;

//Servo details (us)
#define servo_centre 		1500
#define servo_max_lift		1000
#define servo_min_lift		2000
#define motor_min			1000
#define arm_point 1500

// Placeholder varaibles for CPPM and PWM signals
volatile int cppm_channels[9];
byte cppm_channel = 0;
unsigned long cppm_channel_start = 0;
unsigned long cppm_channel_end = 1;
unsigned int pwm_out[2];

// HMC placeholders:
volatile boolean hmc_data_ready = false;
uint8_t hmc_buffer[6];
int16_t hmc_raw[3];
//Yaw and heading variables:
float heading_target = 0.0;
float heading_target_previous = 0.0;


void setup(){
	//interupt setup
	noInterrupts();
	attachInterrupt(digitalPinToInterrupt(0), cppm_interrupt, RISING);
	attachInterrupt(digitalPinToInterrupt(7), hmc_interrupt, RISING);
	interrupts();
	timer_interrupt_setup();
	hmc_setup();
	

	// Pinmode configs:
	pinMode(servo_pin, OUTPUT);
	pinMode(motor_pin, OUTPUT);
	pinMode(heading_led_pin, OUTPUT);
}

void loop(){

	//if there is new compass data
	if(hmc_data_ready){
		hmc_data_ready = false;
		hmc_read();
	}

	//Asimuth approximation:
	float mag_heading_current = atan2(hmc_raw[0], hmc_raw[2]);
	//Asuming 1000us looptime, this should result in 100dps max yaw
	heading_target = heading_target_previous + fap(cppm_channels[3],1000,2000,-0.003141592654,0.003141592654);
	heading_target_previous = heading_target;
	float heading_relative = relative_heading_calc(mag_heading_current, heading_target);

	//turn on heading LED for rear 60 degrees
	digitalWrite(heading_led_pin, ((heading_relative>2.62) && (heading_relative<3.67)));


	//determine the components of roll pitch and yaw and their impacts on flight
	float component_pitch = (sin(heading_relative)*(cppm_channels[1]-1500)*0.6);
	float component_roll = (cos(heading_relative)*(cppm_channels[0]-1500)*0.6);
	float component_collective = (0.4*cppm_channels[2] - 600);
	int servo_out_temp = (int)max(1000,min(2000,((component_collective + component_roll + component_pitch) + servo_centre)));

	//Checks if system is armed with cppm_channel[5]
	if (cppm_channels[5]>arm_point){
		//update motor and servo outputs
		pwm_out[0] = servo_out_temp;
		pwm_out[1] = cppm_channels[4];
	}else{
		pwm_out[0] = servo_centre;
		pwm_out[1] = motor_min;
	}
	update_outputs();
}


//configure compass
void hmc_setup(){
	uint8_t mag_name;
 
	// make sure that the device is connected
	Wire.beginTransmission(HMC_ADDRESS);
	Wire.write((byte) 0x0A); // Identification Register A
	Wire.endTransmission();
	 
	Wire.beginTransmission(HMC_ADDRESS);
	Wire.requestFrom(HMC_ADDRESS, 1);
	mag_name = Wire.read();
	Wire.endTransmission();


	// Register 0x00: CONFIG_A
	// normal measurement mode (0x00) and 75 Hz ODR (0x18)
	Wire.beginTransmission(HMC_ADDRESS);
	Wire.write((byte) HMC_REG_MAG_CRA_REG_M);
	Wire.write((byte) 0x18);
	Wire.endTransmission();
	delayMicroseconds(5000);
 
	// Register 0x01: CONFIG_B
	// default range of +/- 130 uT (0x20)
	Wire.beginTransmission(HMC_ADDRESS);
	Wire.write((byte) HMC_REG_MAG_CRB_REG_M);
	Wire.write((byte) 0x20);
	Wire.endTransmission();
	delayMicroseconds(5000);
 
	// Register 0x02: MODE
	// continuous measurement mode at configured ODR (0x00)
	// possible to achieve 160 Hz by using single measurement mode (0x01) and DRDY
	Wire.beginTransmission(HMC_ADDRESS);
	Wire.write((byte) HMC_REG_MAG_MR_REG_M);
	Wire.write(0x01);
	Wire.endTransmission();
	delayMicroseconds(5000);
}


void hmc_interrupt(){
	hmc_data_ready = true;
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


void cppm_interrupt(){
	//Initialise timing for next channel
	cppm_channel_end = micros();
	int cppm_channel_duration = (int)(cppm_channel_end - cppm_channel_start);
	cppm_channel_start = cppm_channel_end;
	cppm_channels[cppm_channel] = cppm_channel_duration;
	//Increase or overflow from 9'th channel (frame buffer) to 0;
	if (cppm_channel_duration>2500){
		cppm_channel = 0;
	}else{
		cppm_channel ++;
	}
}

float relative_heading_calc(float current, float target){
	float temp = ((current + target)/ 6.2831853);
	int x = (int)temp;
	float mod = ((current + target) - (x * 6.2831853));
	if(mod<0){
		return (mod + 6.2831853);
	}else{
		return mod;
	}
}

//float Mapping function
float fap(int x, int in_min, int in_max, float out_min, float out_max)
{
  return (((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min);
}


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

	int i = 0;
	while(Wire.available() >0 && i < 6){
		hmc_buffer[i] = Wire.read();  // Read one byte
		i++;
	}
	Wire.read();
	Wire.endTransmission();

	// combine the raw data into full integers (HMC588L sends MSB first)
	//           ________ MSB _______   _____ LSB ____
	hmc_raw[0] = ((int)(hmc_buffer[0] << 8) | hmc_buffer[1]); //x
	hmc_raw[1] = ((int)(hmc_buffer[2] << 8) | hmc_buffer[3]); //z
	hmc_raw[2] = ((int)(hmc_buffer[4] << 8) | hmc_buffer[5]); //y

	// put the device back into single measurement mode
	Wire.beginTransmission(HMC_ADDRESS);
	Wire.write((byte) HMC_REG_MAG_MR_REG_M);
	Wire.write((byte) HMC_REG_MAG_CRB_REG_M);
	Wire.endTransmission();
}