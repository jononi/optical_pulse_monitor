#ifndef _ADAFRUIT_SPITFT_MACROS
#define _ADAFRUIT_SPITFT_MACROS
/*
*
*   THIS IS FOR PARTICLE DEVICES ONLY!!!
*/

/*
 * Control Pins
 * */

#define SPI_DC_HIGH()           digitalWrite(_dc, HIGH)
#define SPI_DC_LOW()            digitalWrite(_dc, LOW)
#define SPI_CS_HIGH()           { if(_cs >= 0) digitalWrite(_cs, HIGH); }
#define SPI_CS_LOW()            { if(_cs >= 0) digitalWrite(_cs, LOW);  }

/*
 * Hardware SPI Macros
 * */

// JB contrib

#define HSPI_BEGIN_TRANSACTION() _spi->beginTransaction(_spi_settings)
#define HSPI_END_TRANSACTION()   _spi->endTransaction()
#define HSPI_READ()              _spi->transfer(0)  // not used
#define HSPI_WRITE(b)            _spi->transfer((uint8_t)(b))
#define HSPI_WRITE16(s)          HSPI_WRITE((s) >> 8); HSPI_WRITE(s)
#define HSPI_WRITE32(l)          HSPI_WRITE((l) >> 24); HSPI_WRITE((l) >> 16); HSPI_WRITE((l) >> 8); HSPI_WRITE(l)
#define HSPI_WRITE_PIXELS(c,l)   for(uint32_t i=0; i<(l); i+=2){ HSPI_WRITE(((uint8_t*)(c))[i+1]); HSPI_WRITE(((uint8_t*)(c))[i]); } 

/* 
*   SOFTWARE SPI MACROS (not used)
*/
#define SSPI_MOSI_HIGH()        digitalWrite(_mosi, HIGH)
#define SSPI_MOSI_LOW()         digitalWrite(_mosi, LOW)
#define SSPI_SCK_HIGH()         digitalWrite(_sclk, HIGH)
#define SSPI_SCK_LOW()          digitalWrite(_sclk, LOW)
#define SSPI_MISO_READ()        digitalRead(_miso)

#define SSPI_BEGIN_TRANSACTION()
#define SSPI_END_TRANSACTION()
#define SSPI_WRITE(v)           spiWrite(v)
#define SSPI_WRITE16(s)         SSPI_WRITE((s) >> 8); SSPI_WRITE(s)
#define SSPI_WRITE32(l)         SSPI_WRITE((l) >> 24); SSPI_WRITE((l) >> 16); SSPI_WRITE((l) >> 8); SSPI_WRITE(l)
#define SSPI_WRITE_PIXELS(c,l)  for(uint32_t i=0; i<(l); i+=2){ SSPI_WRITE(((uint8_t*)(c))[i+1]); SSPI_WRITE(((uint8_t*)(c))[i]); }

/*
*       SPI MACROS
*/

#define SPI_BEGIN()             if(_sclk < 0){_spi->begin(SPI_MODE_MASTER);}
#define SPI_BEGIN_TRANSACTION() if(_sclk < 0){HSPI_BEGIN_TRANSACTION();}
#define SPI_END_TRANSACTION()   if(_sclk < 0){HSPI_END_TRANSACTION();}
#define SPI_WRITE16(s)          if(_sclk < 0){HSPI_WRITE16(s);}else{SSPI_WRITE16(s);}
#define SPI_WRITE32(l)          if(_sclk < 0){HSPI_WRITE32(l);}else{SSPI_WRITE32(l);}
#define SPI_WRITE_PIXELS(c,l)   if(_sclk < 0){HSPI_WRITE_PIXELS(c,l);}else{SSPI_WRITE_PIXELS(c,l);}

#endif // _ADAFRUIT_SPITFT_MACROS

