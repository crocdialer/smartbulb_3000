#ifndef LED_PATH
#define LED_PATH

#include "utils.h"
#include "ColorDefines.h"
#include <Adafruit_NeoPixel_ZeroDMA.h>
// #include <Adafruit_NeoPixel.h>

using LedType = Adafruit_NeoPixel_ZeroDMA;
// using LedType = Adafruit_NeoPixel;

// #define CURRENT_LED_TYPE (NEO_RGB + NEO_KHZ800)
// #define BYTES_PER_PIXEL 3

#define CURRENT_LED_TYPE (NEO_RGBW + NEO_KHZ800)
#define BYTES_PER_PIXEL 4

#define TUBE_LENGTH 24
#define SEGMENT_LENGTH TUBE_LENGTH // (2 tubes, each 58 px)

class Segment
{
public:
    Segment(uint8_t *the_data, uint32_t the_length);
    inline uint32_t length() const {return m_length; }
    inline uint32_t color() const { return m_color; }
    inline void set_color(uint32_t the_color){ m_color = the_color; }
    inline void set_active(bool b){ m_active = b; }
    inline bool active() const{ return m_active; }
    inline uint8_t* data() { return m_data; };
private:
    uint8_t* m_data = nullptr;
    uint32_t m_length;
    uint32_t m_color = AQUA;
    bool m_active = true;
};

class LED_Path
{
public:
    LED_Path(){};
    LED_Path(uint32_t the_pin, uint32_t the_num_segments);
    ~LED_Path();

    inline uint32_t num_leds() const{ return num_segments() * SEGMENT_LENGTH; }
    inline uint32_t num_segments() const{ return m_num_segments; };
    Segment* segment(uint32_t the_index) const;

    void set_all_segments(uint32_t the_color);
    inline float brightness(){ return m_brightness; }
    void set_brightness(float the_brightness);

    void clear();
    void update(uint32_t the_delta_time);

    inline uint8_t* data() { return m_data; };
    inline uint32_t num_bytes() const { return num_leds() * BYTES_PER_PIXEL; };
    inline LedType* strip() { return m_strip; }

    inline uint32_t current_max() const{ return m_current_max; }
    inline void set_current_max(uint32_t the_max){ m_current_max = the_max; }

private:

    uint8_t* m_data = nullptr;
    LedType* m_strip = nullptr;
    uint32_t m_num_segments;
    Segment** m_segments = nullptr;
    float m_brightness = .4f;

    float m_current_max;
};
#endif
