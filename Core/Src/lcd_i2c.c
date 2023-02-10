#include "lcd_i2c.h"

#define LCD_ADDR 0x4E

// in 4 bit mode, the upper 4 bits are sent followed by lower 4 bits
// data in [7:4], [3] is backlight, [2] is enable,  [1] is R/W (1 for read, 0 for write), [0] is data/instruction (1 for data, 0 for instruction)
void lcd_instruction_send(I2C_HandleTypeDef *hi2c, uint8_t cmd){

	uint8_t upper_half, lower_half;
	uint8_t send_data[4];
	upper_half = cmd & 0b11110000;
	lower_half = (cmd << 4) & 0b11110000;
	send_data[0] = upper_half | 0b1100;
	send_data[1] = upper_half | 0b1000;  // enable pin low to complete send
	send_data[2] = lower_half | 0b1100;
	send_data[3] = lower_half | 0b1000;  // enable pin low to complete send

	HAL_I2C_Master_Transmit (hi2c, LCD_ADDR,(uint8_t *) send_data, 4, 100);
}

void lcd_data_send (I2C_HandleTypeDef *hi2c, char data){

	uint8_t upper_half, lower_half;
	uint8_t send_data[4];
	upper_half = data & 0b11110000;
	lower_half = (data << 4) & 0b11110000;
	send_data[0] = upper_half | 0b1101;
	send_data[1] = upper_half | 0b1000;  // enable pin low to complete send
	send_data[2] = lower_half | 0b1101;
	send_data[3] = lower_half | 0b1000;	 // enable pin low to complete send

	HAL_I2C_Master_Transmit (hi2c, LCD_ADDR, send_data, 4, 100);
}

void lcd_send_string (I2C_HandleTypeDef *hi2c, char *str)
{
	while (*str) lcd_data_send (hi2c, *str++);
}

void lcd_init (I2C_HandleTypeDef *hi2c){

	// set into 4 bit mode
	lcd_instruction_send (hi2c, 0b00100000);
	HAL_Delay(1);
	// set function mode (2 line, 5x8 character size)
	lcd_instruction_send(hi2c, 0b00101000);
	HAL_Delay(1);
	// set entry mode (increment, no shift)
	lcd_instruction_send(hi2c, 0b00000110);
	HAL_Delay(1);
	// clear display
	lcd_instruction_send(hi2c, 0b00000001);
	HAL_Delay(1);
	// turn on display
	lcd_instruction_send(hi2c, 0b00001100);
	HAL_Delay(1);
}

void lcd_clear(I2C_HandleTypeDef *hi2c){
	lcd_instruction_send(hi2c, 0b00000001);
}

void lcd_set_pos(I2C_HandleTypeDef *hi2c, int row, int col){

	if (row) col += 64;
	lcd_instruction_send(hi2c, col | 0b10000000);
}
