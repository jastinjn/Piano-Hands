#include "pixy.h"
#include <math.h>



int getDist(SPI_HandleTypeDef *hspi) {
		short width_block = 0;
		short color_code = 0;
		uint8_t dataReceived[31];
		uint8_t getBlocks[] =
		{
		  0xae,
		  0xc1,
		  32,
		  2,
		  255,
		  255
		};

	  HAL_SPI_Transmit(hspi, getBlocks, 6, 10);
	  HAL_SPI_Receive(hspi, dataReceived, 29, 10);

	  color_code = (dataReceived[9] << 8) + dataReceived[10];
	  width_block = (dataReceived[15] << 8) + dataReceived[16];

	  if (color_code == 21248) {
		  return (7.0 * ((9409.0 * 50.0) / 7.0)) / (double)width_block;
	  }
	  return -1;
}
