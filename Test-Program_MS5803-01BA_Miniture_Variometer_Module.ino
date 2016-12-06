#include <stdio.h>
#include <math.h>
#include <SPI.h>

//DRUCKSENSOR
// Declaration of Copyright
// Copyright (c) for pressure sensor 2009 MEAS Switzerland
// Edited 2015 Johann Lange
// Edited 2016 Gerrit Brinkmann
// @brief This C code is for starter reference only. It was based on the
// MEAS Switzerland MS56xx pressure sensor modules and Atmel Atmega644p
// microcontroller code and has been translated by Johann Lange and Gerrit Brinkmann
// to work with Teensy 3.1/3.2 microcontroller.
//Macros
#define TRUE 1
#define FALSE 0
#define CMD_RESET 0x1E //ADC reset command (ADC = Analgo-Digital-Converter)
#define CMD_ADC_READ 0x00 //ADC read command
#define CMD_ADC_CONV 0x40 //ADC conversion command
#define CMD_ADC_D1 0x00 //ADC D1 conversion
#define CMD_ADC_D2 0x10 //ADC D2 conversion
#define CMD_ADC_256 0x00 //ADC OSR=256 (OSR = Oversamplingrate)
#define CMD_ADC_512 0x02 //ADC OSR=512
#define CMD_ADC_1024 0x04 //ADC OSR=1024
#define CMD_ADC_2048 0x06 //ADC OSR=2056
#define CMD_ADC_4096 0x08 //ADC OSR=4096
#define CMD_PROM_RD 0xA0 //Prom read command

//Deklarationen
unsigned long D1; //ADC value of the pressure conversion
unsigned long D2; //ADC value of the temperature conversion
unsigned int C[8]; //calibration coefficients
double P; //compensated pressure value
double T; //compensated temperature value
double dT; //difference between actual and measured temperature
double OFF; //offset at actual temperature
double SENS; //sensitivity at actual temperature
double T2; //compensated pressure value, 2nd order
double OFF2; //compensated pressure value, 2nd order
double SENS2; //compensated pressure value, 2nd order
int a; // integer zum Hochzaehlen der Schleife
unsigned char n_crc;
// int i; //counting variable
int loopnumber = 0; //Anzahl der durchlaufenen Loops
elapsedMillis looptime = 0; //Zeit die der Teensy fuer einen Durchlauf benoetigt in ms



//SPI (Pins des Teensy 3.1/3.2, anschliessen nach Datenblatt des Sensors)
const int MISO_Pin = 12;
const int MOSI_Pin = 11;
const int SCK_Pin = 13;
const int CS_Pin_PS = 10; //Chip Select Pressure Sensor
SPISettings settings_PS(4000000, MSBFIRST, SPI_MODE0); //Grundeinstellungen fuer das SPI-Protokoll, Pressure Sensor

void setup() {

  
 //SPI
  pinMode(CS_Pin_PS, OUTPUT);
  pinMode(MISO_Pin, OUTPUT);
  pinMode(SCK_Pin, OUTPUT);
  pinMode(MOSI_Pin, INPUT);
  SPI.setMOSI(MOSI_Pin);
  SPI.setMISO(MISO_Pin);
  SPI.setSCK(SCK_Pin);
  SPI.begin();

  //Drucksensor
  SPI.beginTransaction(settings_PS);
  cmd_reset(); // reset the module after powerup
  for (int i = 0; i < 8; ++i) {
    C[i] = cmd_prom(i);
    //Serial.printf("C[%i] = %i\n", i, C[i]); // Wenn aktiviert, werden am Anfang die Kalibrierungskoeffs des Drucksensor aufgelistet
  } // read calibration coefficients
  n_crc = crc4(C);
  SPI.endTransaction();

}

void loop() {
  P = berechneDruck();
  a++;
  Serial.println(a);
  Serial.print("P ");
  Serial.println(P);
  Serial.print("T ");
  Serial.println(T);
  delay(500);
  
    // Loopnumber erhoehen
  loopnumber = (loopnumber+1)%2560;
  
    // Looptime zuruecksetzen
  looptime = 0;
  
}

//********************************************************
//
// Druck wird berechnet
//
// Declaration of Copyright
// Copyright (c) 2009 MEAS Switzerland
// Edited 2015 Johann Lange
// This C code is for starter reference only. It was based on the
// MEAS Switzerland MS56xx pressure sensor modules and Atmel Atmega644p
// microcontroller code and has been by translated Johann Lange
// to work with Teensy 3.1 microcontroller.
//
//********************************************************
// Der Drucksensor wird ausgewertet und daraus der Druck bestimmt
double berechneDruck() {
  SPI.beginTransaction(settings_PS);
  // delay required in µs: OSR_4096: 9100, OSR_2048: 4600, OSR_1024: 2300, OSR_512: 1200, OSR_256: 700
  D1 = cmd_adc(CMD_ADC_D1 + CMD_ADC_256, 700); // read uncompensated pressure, Conversation Command + OSR, delay in µs
  D2 = cmd_adc(CMD_ADC_D2 + CMD_ADC_256, 700); // read uncompensated temperature, Conversation Command + OSR, delay in µs

  // calcualte 1st order temperature (MS5803_01b 1st order algorithm), base for 2nd order temperature and pressure
  dT = D2 - C[5] * pow(2, 8); //Serial.print("dT = "); Serial.println(dT);
  OFF = C[2] * pow(2, 16) + dT * C[4] / pow(2, 7); //Serial.print("OFF = "); Serial.println(OFF);
  SENS = C[1] * pow(2, 15) + dT * C[3] / pow(2, 8); //Serial.print("SENS = "); Serial.println(SENS);

  T = (2000 + (dT * C[6]) / pow(2, 23)) / 100;
  P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100;

  // calcualte 2nd order pressure and temperature (MS5803_01b 2nd order algorithm)
  if (T > 20) {
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;

    if (T > 45) {
      SENS2 -= pow(T - 4500, 2) / pow(2, 3);
    }
  }
  else {
    T2 = pow(dT, 2) / pow(2, 31);
    OFF2 = 3 * pow(100 * T - 2000, 2);
    SENS2 = 7 * pow(100 * T - 2000, 2) / pow(2, 3);

    if (T < 15) {
      SENS2 += 2 * pow(100 * T + 1500, 2);
    }
  }

  // Recalculate T, OFF, SENS based on T2, OFF2, SENS2
  T -= T2;
  OFF -= OFF2;
  SENS -= SENS2;

  P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100;
  return P;
  SPI.endTransaction();
}

void cmd_reset(void) {
  digitalWrite(CS_Pin_PS, LOW); // pull CSB low to start the command
  SPI.transfer(CMD_RESET); // send reset sequence
  delay(3); // wait for the reset sequence timing
  digitalWrite(CS_Pin_PS, HIGH); // pull CSB high to finish the command
}

//brief preform adc conversion
//return 24bit result
unsigned long cmd_adc(char cmd, int delaytime) {
  digitalWrite(CS_Pin_PS, LOW);
  unsigned long ret;
  unsigned long temp = 0;
  SPI.transfer(CMD_ADC_CONV + cmd); // send conversion command;
  cmd = SPI.transfer(0x00);
  delayMicroseconds(delaytime); // delay required in µs: OSR_4096: 9100, OSR_2048: 4600, OSR_1024: 2300, OSR_512: 1200, OSR_256: 700
  digitalWrite(CS_Pin_PS, HIGH);// pull CSB high to finish the conversion
  digitalWrite(CS_Pin_PS, LOW); // pull CSB low to start new command
  SPI.transfer(CMD_ADC_READ); // send ADC read command
  ret = SPI.transfer(0x00); // send 0 to read 1st byte (MSB)
  temp = 65536 * ret;
  ret = SPI.transfer(0x00); // send 0 to read 2nd byte
  temp = temp + 256 * ret;
  ret = SPI.transfer(0x00); // send 0 to read 3rd byte (LSB)
  temp = temp + ret;
  digitalWrite(CS_Pin_PS, HIGH); // pull CSB high to finish the read command
  return temp;
}

//brief Read calibration coefficients
//return coefficient
unsigned int cmd_prom(char coef_num) {
  unsigned int ret;
  unsigned int rC = 0;

  digitalWrite(CS_Pin_PS, LOW); // pull CSB low
  SPI.transfer(CMD_PROM_RD + coef_num * 2); // send PROM READ command
  ret = SPI.transfer(0x00); // send 0 to read the MSB
  rC = 256 * ret;
  ret = SPI.transfer(0x00); // send 0 to read the LSB
  rC = rC + ret;
  digitalWrite(CS_Pin_PS, HIGH);// pull CSB high
  return rC;
}

//brief calculate the CRC code for details look into CRC CODE NOTES
//return crc code
unsigned char crc4(unsigned int n_prom[]) {
  int cnt; // simple counter
  unsigned int n_rem; // crc reminder
  unsigned int crc_read; // original value of the crc
  unsigned char n_bit;
  n_rem = 0x00;
  crc_read = n_prom[7]; // save read CRC
  n_prom[7] = (0xFF00 & (n_prom[7])); // CRC byte is replaced by 0
  for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
  { // choose LSB or MSB
    if (cnt % 2 == 1) n_rem ^= (unsigned short) ((n_prom[cnt >> 1]) & 0x00FF);
    else n_rem ^= (unsigned short) (n_prom[cnt >> 1] >> 8);
    for (n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & (0x8000))
      {
        n_rem = (n_rem << 1) ^ 0x3000;
      }
      else
      {
        n_rem = (n_rem << 1);
      }
    }
  }
  n_rem = (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
  n_prom[7] = crc_read; // restore the crc_read to its original place
  return (n_rem ^ 0x00);
}

