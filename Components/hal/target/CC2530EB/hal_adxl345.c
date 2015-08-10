/***************************************************
 *												   	*
 *												   	*
 *		ADXL345	for CC2530						   	*
 *												   	*
 ****************************************************
 *													*
 *	Author: smile boy								*
 *  BuildTime: 2015/4/19							*
 *	Reference:	ADXL345.c ADXL345.h file from 		*
 *				builder.org							*
 *													*
 ***************************************************/
 
// Header
#include "hal_i2c.h"
#include "zcomdef.h"
#include "hal_adxl345.h"
 
#define ADXL345_DEVICE (0x53)    // ADXL345 device address
#define ADXL345_TO_READ (6)      // number of bytes we are going to read each time (two bytes for each axis)

////////////////////////////////////////////////
// Global Variable

// ADXL345 status
bool status = ADXL345_OK;		

// Error code
byte error_code = ADXL345_NO_ERROR;

// gains
// gains[0] = 0.00376390;
// gains[1] = 0.00376009;
// gains[2] = 0.00349265;
double gains[3] = {0.00376390, 0.00376009, 0.00349265};

// buffer
byte _buff[6] = {0x00};

///////////////////////////////////////////////////////////////
// FUNC

void ADXL345_PowerOn() 
{
	//Wire.begin();        // join i2c bus (address optional for master)
  	//hali2cStart();
	//Turning on the ADXL345
	ADXL345_writeTo(ADXL345_POWER_CTL, 0);      
	ADXL345_writeTo(ADXL345_POWER_CTL, 16);
	ADXL345_writeTo(ADXL345_POWER_CTL, 8); 
}

void ADXL345_readAccel(uint8* buffer)
{
  ADXL345_readFrom(ADXL345_DATAX0, ADXL345_TO_READ, buffer);
}

// Reads the acceleration into three variable x, y and z
//void ADXL345_readAccel(uint16 *xyz)
//{
//	ADXL345_readAccel(xyz, xyz + 1, xyz + 2);
//}

//void ADXL345_readAccel(uint16 *x, uint16 *y, uint16 *z) 
//{
//	ADXL345_readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff); //read the acceleration data from the ADXL345
//	
//	// each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
//	// thus we are converting both bytes in to one int
//	*x = (((int)_buff[1]) << 8) | _buff[0];   
//	*y = (((int)_buff[3]) << 8) | _buff[2];
//	*z = (((int)_buff[5]) << 8) | _buff[4];
//}

//void ADXL345_get_Gxyz(double *xyz){
//	int i;
//	int xyz_int[3];
//	ADXL345_readAccel(xyz_int);
//	for(i=0; i<3; i++){
//		xyz[i] = xyz_int[i] * gains[i];
//	}
//}

// Writes val to address register on device
void ADXL345_writeTo(byte address, byte val) 
{
	// Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device 
	// Wire.write(address);             		// send register address
	// Wire.write(val);                 		// send value to write
	// Wire.endTransmission();         		// end transmission
	// use CC2530 defined IO-pins
	ADXL345WriteByte(address, val);
}

// Reads num bytes starting from address register on device in to _buff array
void ADXL345_readFrom(byte address, byte num, byte _buff[]) 
{
	// Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device 
	// Wire.write(address);             		// sends address to read from
	// Wire.endTransmission();         		// end transmission
	
	// Wire.beginTransmission(ADXL345_DEVICE); 	// start transmission to device
	// Wire.requestFrom(ADXL345_DEVICE, num);    	// request 6 bytes from device
	
	// int i = 0;
	// while(Wire.available())         		// device may send less than requested (abnormal)
	// { 
		// _buff[i] = Wire.read();    			// receive a byte
		// i++;
	// }
	// if(i != num){
		// status = ADXL345_ERROR;
		// error_code = ADXL345_READ_ERROR;
	// }
	// Wire.endTransmission();         // end transmission
	// use CC2530 I2C interface
	ADXL345ReadBytes(address, _buff, num);
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

//// Gets the range setting and return it into rangeSetting
//// it can be 2, 4, 8 or 16
//void ADXL345_getRangeSetting(byte* rangeSetting) {
//	byte _b;
//	ADXL345_readFrom(ADXL345_DATA_FORMAT, 1, &_b);
//	*rangeSetting = _b & 0x03;
//}
//
//// Sets the range setting, possible values are: 2, 4, 8, 16
//void ADXL345_setRangeSetting(int val) {
//	byte _s;
//	byte _b;
//	
//	switch (val) {
//		case 2:  
//			_s = 0x00; 
//			break;
//		case 4:  
//			_s = 0x01; 
//			break;
//		case 8:  
//			_s = 0x02; 
//			break;
//		case 16: 
//			_s = 0x03; 
//			break;
//		default: 
//			_s = 0x00;
//	}
//	ADXL345_readFrom(ADXL345_DATA_FORMAT, 1, &_b);
//	_s |= (_b & 0xEC);
//	ADXL345_writeTo(ADXL345_DATA_FORMAT, _s);
//}
//// gets the state of the SELF_TEST bit
//bool ADXL345_getSelfTestBit() {
//	return ADXL345_getRegisterBit(ADXL345_DATA_FORMAT, 7);
//}
//
//// Sets the SELF-TEST bit
//// if set to 1 it applies a self-test force to the sensor causing a shift in the output data
//// if set to 0 it disables the self-test force
//void ADXL345_setSelfTestBit(bool selfTestBit) {
//	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
//}
//
//// Gets the state of the SPI bit
//bool ADXL345_getSpiBit() {
//	return ADXL345_getRegisterBit(ADXL345_DATA_FORMAT, 6);
//}
//
//// Sets the SPI bit
//// if set to 1 it sets the device to 3-wire mode
//// if set to 0 it sets the device to 4-wire SPI mode
//void ADXL345_setSpiBit(bool spiBit) {
//	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
//}
//
//// Gets the state of the INT_INVERT bit
//bool ADXL345_getInterruptLevelBit() {
//	return ADXL345_getRegisterBit(ADXL345_DATA_FORMAT, 5);
//}
//
//// Sets the INT_INVERT bit
//// if set to 0 sets the interrupts to active high
//// if set to 1 sets the interrupts to active low
//void ADXL345_setInterruptLevelBit(bool interruptLevelBit) {
//	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
//}
//
//// Gets the state of the FULL_RES bit
//bool ADXL345_getFullResBit() {
//	return ADXL345_getRegisterBit(ADXL345_DATA_FORMAT, 3);
//}
//
//// Sets the FULL_RES bit
//// if set to 1, the device is in full resolution mode, where the output resolution increases with the
////   g range set by the range bits to maintain a 4mg/LSB scal factor
//// if set to 0, the device is in 10-bit mode, and the range buts determine the maximum g range
////   and scale factor
//void ADXL345_setFullResBit(bool fullResBit) 
//{
//	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
//}
//
//// Gets the state of the justify bit
//bool ADXL345_getJustifyBit() 
//{
//	return ADXL345_getRegisterBit(ADXL345_DATA_FORMAT, 2);
//}
//
//// Sets the JUSTIFY bit
//// if sets to 1 selects the left justified mode
//// if sets to 0 selects right justified mode with sign extension
//void ADXL345_setJustifyBit(bool justifyBit) {
//	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
//}
//
//// Sets the THRESH_TAP byte value
//// it should be between 0 and 255
//// the scale factor is 62.5 mg/LSB
//// A value of 0 may result in undesirable behavior
//void ADXL345_setTapThreshold(int tapThreshold) {
//	tapThreshold = constrain(tapThreshold,0,255);
//	byte _b = byte (tapThreshold);
//	ADXL345_writeTo(ADXL345_THRESH_TAP, _b);  
//}
//
//// Gets the THRESH_TAP byte value
//// return value is comprised between 0 and 255
//// the scale factor is 62.5 mg/LSB
//int ADXL345_getTapThreshold() {
//	byte _b;
//	ADXL345_readFrom(ADXL345_THRESH_TAP, 1, &_b);  
//	return int(_b);
//}
//
//// set/get the gain for each axis in Gs / count
//void ADXL345_setAxisGains(double *_gains){
//	int i;
//	for(i = 0; i < 3; i++){
//		gains[i] = _gains[i];
//	}
//}
//void ADXL345_getAxisGains(double *_gains){
//	int i;
//	for(i = 0; i < 3; i++){
//		_gains[i] = gains[i];
//	}
//}
//
//
//// Sets the OFSX, OFSY and OFSZ bytes
//// OFSX, OFSY and OFSZ are user offset adjustments in twos complement format with
//// a scale factor of 15,6mg/LSB
//// OFSX, OFSY and OFSZ should be comprised between 
//void ADXL345_setAxisOffset(int x, int y, int z) {
//	ADXL345_writeTo(ADXL345_OFSX, byte (x));  
//	ADXL345_writeTo(ADXL345_OFSY, byte (y));  
//	ADXL345_writeTo(ADXL345_OFSZ, byte (z));  
//}
//
//// Gets the OFSX, OFSY and OFSZ bytes
//void ADXL345_getAxisOffset(int* x, int* y, int*z) {
//	byte _b;
//	ADXL345_readFrom(ADXL345_OFSX, 1, &_b);  
//	*x = int (_b);
//	ADXL345_readFrom(ADXL345_OFSY, 1, &_b);  
//	*y = int (_b);
//	ADXL345_readFrom(ADXL345_OFSZ, 1, &_b);  
//	*z = int (_b);
//}
//
//// Sets the DUR byte
//// The DUR byte contains an unsigned time value representing the maximum time
//// that an event must be above THRESH_TAP threshold to qualify as a tap event
//// The scale factor is 625¦Ìs/LSB
//// A value of 0 disables the tap/double tap funcitons. Max value is 255.
//void ADXL345_setTapDuration(int tapDuration) {
//	tapDuration = constrain(tapDuration,0,255);
//	byte _b = byte (tapDuration);
//	ADXL345_writeTo(ADXL345_DUR, _b);  
//}
//
//// Gets the DUR byte
//int ADXL345_getTapDuration() {
//	byte _b;
//	ADXL345_readFrom(ADXL345_DUR, 1, &_b);  
//	return int (_b);
//}
//
//// Sets the latency (latent register) which contains an unsigned time value
//// representing the wait time from the detection of a tap event to the start
//// of the time window, during which a possible second tap can be detected.
//// The scale factor is 1.25ms/LSB. A value of 0 disables the double tap function.
//// It accepts a maximum value of 255.
//void ADXL345_setDoubleTapLatency(int doubleTapLatency) {
//	byte _b = byte (doubleTapLatency);
//	ADXL345_writeTo(ADXL345_LATENT, _b);  
//}
//
//// Gets the Latent value
//int ADXL345_getDoubleTapLatency() {
//	byte _b;
//	ADXL345_readFrom(ADXL345_LATENT, 1, &_b);  
//	return int (_b);
//}
//
//// Sets the Window register, which contains an unsigned time value representing
//// the amount of time after the expiration of the latency time (Latent register)
//// during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
//// value of 0 disables the double tap function. The maximum value is 255.
//void ADXL345_setDoubleTapWindow(int doubleTapWindow) {
//	doubleTapWindow = constrain(doubleTapWindow,0,255);
//	byte _b = byte (doubleTapWindow);
//	ADXL345_writeTo(ADXL345_WINDOW, _b);  
//}
//
//// Gets the Window register
//int ADXL345_getDoubleTapWindow() {
//	byte _b;
//	ADXL345_readFrom(ADXL345_WINDOW, 1, &_b);  
//	return int (_b);
//}
//
//// Sets the THRESH_ACT byte which holds the threshold value for detecting activity.
//// The data format is unsigned, so the magnitude of the activity event is compared 
//// with the value is compared with the value in the THRESH_ACT register. The scale
//// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the 
//// activity interrupt is enabled. The maximum value is 255.
//void ADXL345_setActivityThreshold(int activityThreshold) {
//	activityThreshold = constrain(activityThreshold,0,255);
//	byte _b = byte (activityThreshold);
//	ADXL345_getFreeFallDurationwriteTo(ADXL345_THRESH_ACT, _b);  
//}
//
//// Gets the THRESH_ACT byte
//int ADXL345_getActivityThreshold() {
//	byte _b;
//	ADXL345_readFrom(ADXL345_THRESH_ACT, 1, &_b);  
//	return int (_b);
//}
//
//// Sets the THRESH_INACT byte which holds the threshold value for detecting inactivity.
//// The data format is unsigned, so the magnitude of the inactivity event is compared 
//// with the value is compared with the value in the THRESH_INACT register. The scale
//// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the 
//// inactivity interrupt is enabled. The maximum value is 255.
//void ADXL345_setInactivityThreshold(int inactivityThreshold) {
//	inactivityThreshold = constrain(inactivityThreshold,0,255);
//	byte _b = byte (inactivityThreshold);
//	writeTo(ADXL345_THRESH_INACT, _b);  
//}
//
//// Gets the THRESH_INACT byte
//int ADXL345_getInactivityThreshold() {
//	byte _b;
//	readFrom(ADXL345_THRESH_INACT, 1, &_b);  
//	return int (_b);
//}
//
//// Sets the TIME_INACT register, which contains an unsigned time value representing the
//// amount of time that acceleration must be less thant the value in the THRESH_INACT
//// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
//// be between 0 and 255.
//void ADXL345_setTimeInactivity(int timeInactivity) {
//	timeInactivity = constrain(timeInactivity,0,255);
//	byte _b = byte (timeInactivity);
//	writeTo(ADXL345_TIME_INACT, _b);  
//}
//
//// Gets the TIME_INACT register
//int ADXL345_getTimeInactivity() {
//	byte _b;
//	readFrom(ADXL345_TIME_INACT, 1, &_b);  
//	return int (_b);
//}
//
//// Sets the THRESH_FF register which holds the threshold value, in an unsigned format, for
//// free-fall detection. The root-sum-square (RSS) value of all axes is calculated and
//// compared whith the value in THRESH_FF to determine if a free-fall event occured. The 
//// scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the free-fall
//// interrupt is enabled. The maximum value is 255.
//void ADXL345_setFreeFallThreshold(int freeFallThreshold) {
//	freeFallThreshold = constrain(freeFallThreshold,0,255);
//	byte _b = byte (freeFallThreshold);
//	writeTo(ADXL345_THRESH_FF, _b);  
//}
//
//// Gets the THRESH_FF register.
//int ADXL345_getFreeFallThreshold() 
//{
//	byte _b;
//	readFrom(ADXL345_THRESH_FF, 1, &_b);  
//	return int (_b);
//}
//
//// Sets the TIME_FF register, which holds an unsigned time value representing the minimum
//// time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall 
//// interrupt. The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
//// the free-fall interrupt is enabled. The maximum value is 255.
//void ADXL345_setFreeFallDuration(int freeFallDuration) {
//	freeFallDuration = constrain(freeFallDuration,0,255);  
//	byte _b = byte (freeFallDuration);
//	ADXL345_writeTo(ADXL345_TIME_FF, _b);  
//}
//
//// Gets the TIME_FF register.
//int ADXL345_getFreeFallDuration() 
//{
//	byte _b;
//	ADXL345_readFrom(ADXL345_TIME_FF, 1, &_b);  
//	return int (_b);
//}
//
//bool ADXL345_isActivityXEnabled() 
//{  
//	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 6); 
//}
//bool ADXL345_isActivityYEnabled() {  
//	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 5); 
//}
//bool ADXL345_isActivityZEnabled() {  
//	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 4); 
//}
//bool ADXL345_isInactivityXEnabled() {  
//	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 2); 
//}
//bool ADXL345_isInactivityYEnabled() {  
//	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 1); 
//}
//bool ADXL345_isInactivityZEnabled() {  
//	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 0); 
//}
//
//void ADXL345_setActivityX(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state); 
//}
//void ADXL345_setActivityY(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state); 
//}
//void ADXL345_setActivityZ(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state); 
//}
//void ADXL345_setInactivityX(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state); 
//}
//void ADXL345_setInactivityY(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state); 
//}
//void ADXL345_setInactivityZ(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state); 
//}
//
//bool ADXL345_isActivityAc() { 
//	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 7); 
//}
//bool ADXL345_isInactivityAc(){ 
//	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 3); 
//}
//
//void ADXL345_setActivityAc(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state); 
//}
//void ADXL345_setInactivityAc(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 3, state); 
//}
//
//bool ADXL345_getSuppressBit(){ 
//	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 3); 
//}
//void ADXL345_setSuppressBit(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 3, state); 
//}
//
//bool ADXL345_isTapDetectionOnX(){ 
//	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 2); 
//}
//void ADXL345_setTapDetectionOnX(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 2, state); 
//}
//bool ADXL345_isTapDetectionOnY(){ 
//	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 1); 
//}
//void ADXL345_setTapDetectionOnY(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 1, state); 
//}
//bool ADXL345_isTapDetectionOnZ(){ 
//	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 0); 
//}
//void ADXL345_setTapDetectionOnZ(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 0, state); 
//}
//
//bool ADXL345_isActivitySourceOnX(){ 
//	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 6); 
//}
//bool ADXL345_isActivitySourceOnY(){ 
//	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 5); 
//}
//bool ADXL345_isActivitySourceOnZ(){ 
//	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 4); 
//}
//
//bool ADXL345_isTapSourceOnX(){ 
//	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 2); 
//}
//bool ADXL345_isTapSourceOnY(){ 
//	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 1); 
//}
//bool ADXL345_isTapSourceOnZ(){ 
//	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 0); 
//}
//
//bool ADXL345_isAsleep(){ 
//	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 3); 
//}
//
//bool ADXL345_isLowPower(){ 
//	return ADXL345_getRegisterBit(ADXL345_BW_RATE, 4); 
//}
//void ADXL345_setLowPower(bool state) {  
//	ADXL345_setRegisterBit(ADXL345_BW_RATE, 4, state); 
//}
//
//double ADXL345_getRate(){
//	byte _b;
//	ADXL345_readFrom(ADXL345_BW_RATE, 1, &_b);
//	_b &= B00001111;
//	return (pow(2,((int) _b)-6)) * 6.25;
//}
//
//void ADXL345_setRate(double rate){
//	byte _b,_s;
//	int v = (int) (rate / 6.25);
//	int r = 0;
//	while (v >>= 1)
//	{
//		r++;
//	}
//	if (r <= 9) { 
//		ADXL345_readFrom(ADXL345_BW_RATE, 1, &_b);
//		_s = (byte) (r + 6) | (_b & B11110000);
//		ADXL345_writeTo(ADXL345_BW_RATE, _s);
//	}
//}
//
//void ADXL345_set_bw(byte bw_code){
//	if((bw_code < ADXL345_BW_3) || (bw_code > ADXL345_BW_1600)){
//		status = false;
//		error_code = ADXL345_BAD_ARG;
//	}
//	else{
//		ADXL345_writeTo(ADXL345_BW_RATE, bw_code);
//	}
//}
//
//byte ADXL345_get_bw_code(){
//	byte bw_code;
//	ADXL345_readFrom(ADXL345_BW_RATE, 1, &bw_code);
//	return bw_code;
//}
//
//
//
//
//
////Used to check if action was triggered in interrupts
////Example triggered(interrupts, ADXL345_SINGLE_TAP);
//bool ADXL345_triggered(byte interrupts, int mask){
//	return ((interrupts >> mask) & 1);
//}
//
//
///*
// ADXL345_DATA_READY
// ADXL345_SINGLE_TAP
// ADXL345_DOUBLE_TAP
// ADXL345_ACTIVITY
// ADXL345_INACTIVITY
// ADXL345_FREE_FALL
// ADXL345_WATERMARK
// ADXL345_OVERRUNY
// */
//
//
//
//
//
//byte ADXL345_getInterruptSource() {
//	byte _b;
//	ADXL345_readFrom(ADXL345_INT_SOURCE, 1, &_b);
//	return _b;
//}
//
//bool ADXL345_getInterruptSource(byte interruptBit) {
//	return ADXL345_getRegisterBit(ADXL345_INT_SOURCE,interruptBit);
//}
//
//bool ADXL345_getInterruptMapping(byte interruptBit) {
//	return ADXL345_getRegisterBit(ADXL345_INT_MAP,interruptBit);
//}
//
//// Set the mapping of an interrupt to pin1 or pin2
//// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
//void ADXL345_setInterruptMapping(byte interruptBit, bool interruptPin) {
//	ADXL345_setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
//}
//
//bool ADXL345_isInterruptEnabled(byte interruptBit) {
//	return ADXL345_getRegisterBit(ADXL345_INT_ENABLE,interruptBit);
//}
//
//void ADXL345_setInterrupt(byte interruptBit, bool state) {
//	ADXL345_setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
//}
//
//void ADXL345_setRegisterBit(byte regAdress, int bitPos, bool state) {
//	byte _b;
//	ADXL345_readFrom(regAdress, 1, &_b);
//	if (state) {
//		_b |= (1 << bitPos);  // forces nth bit of _b to be 1.  all other bits left alone.
//	} 
//	else {
//		_b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
//	}
//	ADXL345_writeTo(regAdress, _b);  
//}
//
//bool ADXL345_getRegisterBit(byte regAdress, int bitPos) {
//	byte _b;
//	ADXL345_readFrom(regAdress, 1, &_b);
//	return ((_b >> bitPos) & 1);
//}


