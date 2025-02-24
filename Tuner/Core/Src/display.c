/*
 * display.c
 *
 *  Created on: Feb 8, 2025
 *      Author: evant
 */
#define __FPU_PRESENT  1U

#include <stdlib.h>
#include "arm_math.h"
#include "display.h"
#include "st7735.h"
#include "images.h"

/* Display Parameters */
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 160
#define PADDING 10
/* Bar Parameters */
#define BAR_HEIGHT 40
#define BAR_WIDTH 6
#define GAP_WIDTH 1
/* Note Parameters */
#define NOTE_WIDTH 56
#define NOTE_HEIGHT 84
#define SHARP_WIDTH 20
#define SHARP_HEIGHT 43

int currNote, prevNote;

void display_init() {
  ST7735_FillScreenFast(ST7735_BLACK);
  ST7735_FillRectangleFast(DISPLAY_WIDTH/2 - BAR_WIDTH/2, PADDING, BAR_WIDTH, BAR_HEIGHT, ST7735_YELLOW);
}

int closest_note(float32_t pitch) {
    pitch = roundf(fmodf(pitch, 12.0f));
    return ((int)pitch + 12) % 12;
}

void draw_note(const uint16_t* note, uint8_t isSharp) {
  ST7735_DrawImage((DISPLAY_WIDTH - NOTE_WIDTH)/2, DISPLAY_HEIGHT - NOTE_HEIGHT - PADDING, NOTE_WIDTH, NOTE_HEIGHT, note);
  if (isSharp) {
    ST7735_DrawImage((DISPLAY_WIDTH - NOTE_WIDTH)/2 + NOTE_WIDTH, DISPLAY_HEIGHT - NOTE_HEIGHT - PADDING, SHARP_WIDTH, SHARP_HEIGHT, image_data_Font_0x23);
  } else {
    ST7735_FillRectangleFast((DISPLAY_WIDTH - NOTE_WIDTH)/2 + NOTE_WIDTH, DISPLAY_HEIGHT - NOTE_HEIGHT - PADDING, SHARP_WIDTH, SHARP_HEIGHT, ST7735_BLACK);
  }
}

void draw_bar(float32_t cents){
  // Quantize cents
  int16_t q = (cents + BAR_WIDTH/2.0) / BAR_WIDTH;

  // Initialize offset
  int16_t x = (DISPLAY_WIDTH - (2 * (BAR_WIDTH + GAP_WIDTH) * (50/BAR_WIDTH) + BAR_WIDTH)) / 2;

  // Draw bar
  for (int8_t i = -50/BAR_WIDTH; i < 0; i++) {
    if (i <= q) {
      ST7735_FillRectangleFast(x, PADDING, BAR_WIDTH, BAR_HEIGHT, ST7735_RED);
    }
    else {
      ST7735_FillRectangleFast(x, PADDING, BAR_WIDTH, BAR_HEIGHT, ST7735_BLACK);
    }
    x += BAR_WIDTH + GAP_WIDTH;
  }

  // Draw center
  if (q >= 0){
    ST7735_FillRectangleFast(x, PADDING, BAR_WIDTH, BAR_HEIGHT, ST7735_YELLOW);
  }
  else {
    ST7735_FillRectangleFast(x, PADDING, BAR_WIDTH, BAR_HEIGHT, ST7735_BLACK);
  }
  x += BAR_WIDTH;

  // Draw sharp bar
  for (int8_t i = 0; i < 50/BAR_WIDTH; i++) {
    x += GAP_WIDTH;
    if (i < q ) {
      ST7735_FillRectangleFast(x, PADDING, BAR_WIDTH, BAR_HEIGHT, ST7735_GREEN);
    }
    else {
      ST7735_FillRectangleFast(x, PADDING, BAR_WIDTH, BAR_HEIGHT, ST7735_BLACK);
    }
    x += BAR_WIDTH;
  }
}

void update_display(float32_t pitch) {
  draw_bar((pitch - roundf(pitch)) * 100.0f);

  currNote = closest_note(pitch);

  // Only draw note when note changes
  if (currNote != prevNote) {
    switch (currNote)
    {
    case 0:
      draw_note(image_data_Font_0x41, 0);
      break;
    case 1:
      draw_note(image_data_Font_0x41, 1);
      break;
    case 2:
      draw_note(image_data_Font_0x42, 0);
      break;
    case 3:
      draw_note(image_data_Font_0x43, 0);
      break;
    case 4:
      draw_note(image_data_Font_0x43, 1);
      break;
    case 5:
      draw_note(image_data_Font_0x44, 0);
      break;
    case 6:
      draw_note(image_data_Font_0x44, 1);
      break;
    case 7:
      draw_note(image_data_Font_0x45, 0);
      break;
    case 8:
      draw_note(image_data_Font_0x46, 0);
      break;
    case 9:
      draw_note(image_data_Font_0x46, 1);
      break;
    case 10:
      draw_note(image_data_Font_0x47, 0);
      break;
    case 11:
      draw_note(image_data_Font_0x47, 1);
      break;
    }
  }

  prevNote = currNote;
}
