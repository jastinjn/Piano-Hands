/* midi_synth.h
 *
 *
 *
 */

#ifndef INC_MIDI_SYNTH_H_
#define INC_MIDI_SYNTH_H_
#include "stm32f4xx_hal.h"


void playNote(UART_HandleTypeDef* uart, uint8_t note_value, uint8_t striking_velocity);
void stopNote(UART_HandleTypeDef* uart, uint8_t note_value, uint8_t striking_velocity);
void changeVolume(UART_HandleTypeDef *uart, uint8_t volume);
void changeInstrument(UART_HandleTypeDef *uart, uint8_t instrument);


#endif /* INC_MIDI_SYNTH_H_ */
