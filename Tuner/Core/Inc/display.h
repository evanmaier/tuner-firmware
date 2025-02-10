/*
 * display.h
 *
 *  Created on: Feb 8, 2025
 *      Author: evant
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

int closest_note(float32_t pitch);
int deviation_cents(float32_t pitch);
void draw_note(const uint16_t* note, uint8_t isSharp);
void draw_bar(float32_t cents);
void update_display(float32_t pitch);
void display_init();

#endif /* INC_DISPLAY_H_ */
