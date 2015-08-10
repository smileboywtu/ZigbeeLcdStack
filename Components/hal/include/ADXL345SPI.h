#ifndef SPI_ADXL345
#define SPI_ADXL345

void init_port(void);
void init_Baudrate(void);
void Init_Spi(void);
void spiWait(unsigned char counter);
void InitADXL345( void );
void ADXL345Write(unsigned char multi, unsigned char len, unsigned char* buffer, unsigned char startReg);
void ADXL345Read(unsigned char multi, unsigned char len, unsigned char* buffer, unsigned char startReg);


#endif