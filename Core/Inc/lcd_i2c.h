#include "stm32f4xx_hal.h"

void lcd_instruction_send(I2C_HandleTypeDef *hi2c, uint8_t cmd);
void lcd_data_send (I2C_HandleTypeDef *hi2c, char data);
void lcd_send_string (I2C_HandleTypeDef *hi2c, char *str);
void lcd_init (I2C_HandleTypeDef *hi2c);
void lcd_clear(I2C_HandleTypeDef *hi2c);
void lcd_set_pos(I2C_HandleTypeDef *hi2c, int row, int col);
