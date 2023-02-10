


#include "midi_synth.h"

void playNote(UART_HandleTypeDef* uart, uint8_t note_value, uint8_t striking_velocity){

	uint8_t midi_data[] =
	{
			0x90, note_value, striking_velocity,
	};
	HAL_UART_Transmit(uart, midi_data, 3, 10);
}

void stopNote(UART_HandleTypeDef* uart, uint8_t note_value, uint8_t striking_velocity){

	uint8_t midi_data[] =
	{
			0x80, note_value, striking_velocity,
	};
	HAL_UART_Transmit(uart, midi_data, 3, 10);
}

//not working
void changeVolume(UART_HandleTypeDef *uart, uint8_t volume){
	uint8_t midi_data[] =
	{
		0xB0, 0x07, volume
	};
	HAL_UART_Transmit(uart, midi_data, 3, 10);
}

void changeInstrument(UART_HandleTypeDef *uart, uint8_t instrument){
	uint8_t midi_data[] = {
		0xC0, instrument
	};
	HAL_UART_Transmit(uart, midi_data, 2, 10);
}
