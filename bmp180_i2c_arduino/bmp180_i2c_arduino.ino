#include <Wire.h>

// Define global variables
  // I2C address of BMP180
int i2c_addr = 0x77;

  // Calibration variables from EEPROM
int16_t CALIB_AC1;
int16_t CALIB_AC2;
int16_t CALIB_AC3;
uint16_t CALIB_AC4;
uint16_t CALIB_AC5;
uint16_t CALIB_AC6;
int16_t CALIB_B1;
int16_t CALIB_B2;
int16_t CALIB_MB;
int16_t CALIB_MC;
int16_t CALIB_MD;

  // Compensation variables
long _B5;

  // Sampling mode variable
byte _samplingMode;

  // Register addresses
byte TEMP_READ = 0x2E;
byte PRESS_READ = 0x34;
byte CONTROL_REG = 0xF4;
byte MSB = 0xF6;
byte LSB = 0xF7;
byte XLSB = 0xF8;

  // resolutions
byte BMP180_OVERSAMPLING_ULTRA_LOW_POWER = 0x00;
byte BMP180_OVERSAMPLING_STANDARD = 0x01;
byte BMP180_OVERSAMPLING_HIGH_RESOLUTION = 0x02;
byte BMP180_OVERSAMPLING_ULTRA_HIGH_RESOLUTION = 0x03;

// Register select function
void selectRegister(byte reg){
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.endTransmission();
}

// Read byte from register
byte readByteFromRegister(byte reg){
  byte b;
  selectRegister(reg);
  Wire.beginTransmission(i2c_addr);
  Wire.requestFrom(i2c_addr, 1);
  b = Wire.read();
  Wire.endTransmission();
  return b;
}

// Read word from register
uint16_t readWordFromRegister(byte reg){
  uint16_t word;
  selectRegister(reg);
  Wire.beginTransmission(i2c_addr);
  Wire.requestFrom(i2c_addr, 2);
  word = Wire.read();
  word = word << 8 | Wire.read();
  Wire.endTransmission();
  return word;
}

// Read long from register
unsigned long readLongFromRegister(byte reg){
  unsigned long l;
  selectRegister(reg);
  Wire.beginTransmission(i2c_addr);
  Wire.requestFrom(i2c_addr, 3);
  l = Wire.read();
  l = l << 8 | Wire.read();
  l = l << 8 | Wire.read();
  Wire.endTransmission();
  return l;
}

// Measure function
void measure(byte measureID){
  Wire.beginTransmission(i2c_addr);
  Wire.write(CONTROL_REG);
  Wire.write(measureID);
  Wire.endTransmission();
}

//Measure Raw Temperature
long measureTemperature() {
  measure(TEMP_READ);
  delay(5);
  return (long)readWordFromRegister(MSB);
}

// Measure Raw Pressure using oversampling
long measurePressure(byte oversampling){
  measure(PRESS_READ | (oversampling << 6));
  switch(oversampling) {
    case 0: delay(5); break;
    case 1: delay(8); break;
    case 2: delay(14); break;
    case 3: delay(26); break;
  }
  long p = (long) readLongFromRegister(MSB);
  p = p >> (8 - oversampling);
  return p;
}

// Calculate B5 coefficient for temperature
long calculateB5(long UT){
  long X1 = (UT - (long)CALIB_AC6) * (long)CALIB_AC5 >> 15;
  // Serial.println();
  // Serial.print("X1= ");
  // Serial.println(X1);
  long X2 = ((long)CALIB_MC << 11) / (X1 + (long)CALIB_MD);
  // Serial.println();
  // Serial.print("X2= ");
  // Serial.println(X2);

  // Serial.println();
  // Serial.print("X1+X2= ");
  // Serial.println(X1+X2);
  delay(2000);
  return X1 + X2;
}

// Compensate temperature
long compensateTemperature(long UT){
  _B5 = calculateB5(UT);
  return (_B5 + 8) >> 4;
}

// Compensate Pressure
long compensatePressure(long UP, long oversampling){
  long B6, X1, X2, X3, B3, p;
  unsigned long B4, B7;
  B6 = _B5 - 4000;
  X1 = (CALIB_B2 * (B6 * B6 >> 12)) >> 11;
  X2 = (CALIB_AC2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((CALIB_AC1 << 2) + X3) << oversampling) + 2) >> 2;
  X1 = CALIB_AC3 * B6 >> 13;
  X2 = (CALIB_B1 * (B6 * B6 >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = CALIB_AC4 * (unsigned long)(X3 + 32768) >> 15;
  B7 = ((unsigned long)UP - B3) * (50000 >> oversampling);
  if(B7 < 0x80000000){
    p = (B7 << 1) / B4;
  } else {
    p = (B7 / B4) << 1;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + 3791) >> 4);
  return p;
}

// Format temperature for output
float formatTemperature(long T){
  return (float)T / 10;
}

// Format pressure for output
float formatPressure(long P){
  return (float)P / 100;
}

// Get temperature function
float getTemperature(){
  int ut = measureTemperature();
  long t = compensateTemperature(ut);
  return formatTemperature(t);
}

// Get pressure function
float getPressure(){
  long up = measurePressure(_samplingMode);
  long p = compensatePressure(up, _samplingMode);
  return formatPressure(p);
}

// Set sampling mode
void setSamplingMode(byte samplingMode){
  _samplingMode = samplingMode;
}

void setup() {
  // Start I2C
  Wire.begin();
  Wire.setClock(400000);
  // Start serial monitor
  Serial.begin(9600);
  delay(100);

  // Read the calibration parameters
  Wire.beginTransmission(i2c_addr);
  Wire.write(0xAA);
  Wire.endTransmission(false);
  byte buff[22];
  Wire.requestFrom(i2c_addr, 22);
  Wire.readBytes(buff, 22);
  Wire.endTransmission();

  CALIB_AC1 = makeWord(buff[0], buff[1]);
  CALIB_AC2 = makeWord(buff[2], buff[3]);
  CALIB_AC3 = makeWord(buff[4], buff[5]);
  CALIB_AC4 = makeWord(buff[6], buff[7]);
  CALIB_AC5 = makeWord(buff[8], buff[9]);
  CALIB_AC6 = makeWord(buff[10], buff[11]);
  CALIB_B1 = makeWord(buff[12], buff[13]);
  CALIB_B2 = makeWord(buff[14], buff[15]);
  CALIB_MB = makeWord(buff[16], buff[17]);
  CALIB_MC = makeWord(buff[18], buff[19]);
  CALIB_MD = makeWord(buff[20], buff[21]);
  
  Serial.println(CALIB_AC1, HEX);
  for(int i = 0; i < 22; i++){
    Serial.println(buff[i], HEX);
  }

}

void loop() {

  // Set sampling mode
  setSamplingMode(BMP180_OVERSAMPLING_ULTRA_HIGH_RESOLUTION);
  float T = getTemperature();
  delay(50);
  float P = getPressure();

  Serial.println();
  Serial.print("Temperature = ");
  Serial.print(T, 1);
  Serial.println(" C");
  
  Serial.println();
  Serial.print("Pressure = ");
  Serial.print(P);
  Serial.println(" hPa");


  delay(10);
}
