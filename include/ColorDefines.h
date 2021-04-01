#pragma once

#include <Adafruit_NeoPixel.h>

#define COLOR_RGBW
// #define COLOR_BRG

static const uint32_t
#if defined(COLOR_RGBW)
WHITE = Adafruit_NeoPixel::Color(0, 0, 0, 255),
ORANGE = Adafruit_NeoPixel::Color(0, 50, 255, 40),
GREEN = Adafruit_NeoPixel::Color(60, 50, 255, 40),
AQUA = Adafruit_NeoPixel::Color(255, 120, 80, 40),
PURPLE_ISH = Adafruit_NeoPixel::Color(255, 140, 70, 40),
DARK_ORANGE = Adafruit_NeoPixel::Color(0, 200, 40, 40),
TOKYO_CHERRY = Adafruit_NeoPixel::Color(60, 225, 90, 40),

#elif defined(COLOR_BRG)
ORANGE = Adafruit_NeoPixel::Color(0, 255, 50),
GREEN = Adafruit_NeoPixel::Color(50, 60, 255),
AQUA = Adafruit_NeoPixel::Color(255, 80, 120),

#endif

ALL_IN = Adafruit_NeoPixel::Color(255, 255, 255, 255),
BLACK = 0;

static const uint32_t g_colors[] = {ORANGE};
static const uint32_t g_num_colors = 1;
