#pragma once

#include "Arduino.h"

class no_interrupt
{
public:
    no_interrupt(){ noInterrupts(); }
    ~no_interrupt(){ interrupts(); }
};

template <typename T>
void fmt_real_to_str(char *buf, T val, uint32_t precision = 3)
{
    int multiplier = 1;
    char fmt_string[8];
    // sprintf(fmt_string, "%d.%03d")
    for(int i = 0; i < precision; i++){ multiplier *= 10; }
    T frac = val - (int)val;
    frac = frac < 0 ? -frac : frac;
    int32_t fmt = frac * multiplier;
    sprintf(buf, "%d.%03d", (int)val, fmt);
};

template <typename T>
inline int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

template <typename T>
inline T random(const T &min, const T &max)
{
    return min + (max - min) * (rand() / (float) RAND_MAX);
}

template <typename T>
inline const T& clamp(const T &val, const T &min, const T &max)
{
    return val < min ? min : (val > max ? max : val);
}

template <typename T>
inline T mix(const T &lhs, const T &rhs, float ratio)
{
    return lhs + ratio * (rhs - lhs);
}

template <typename T>
inline T map_value(const T &val, const T &src_min, const T &src_max,
                   const T &dst_min, const T &dst_max)
{
    float mix_val = clamp<float>((val - src_min) / (src_max - src_min), 0.f, 1.f);
    return mix<T>(dst_min, dst_max, mix_val);
}

template <typename T>
inline void swap(T& lhs, T& rhs)
{
    T tmp = lhs;
    lhs = rhs;
    rhs = tmp;
}

inline static unsigned int hash(unsigned int x)
{
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = (x >> 16) ^ x;
    return x;
}

/*! smoothstep performs smooth Hermite interpolation between 0 and 1,
 *  when edge0 < x < edge1.
 *  This is useful in cases where a threshold function with a smooth transition is desired
 */
inline float smoothstep(float edge0, float edge1, float x)
{
    float t = clamp<float>((x - edge0) / (edge1 - edge0), 0.f, 1.f);
    return t * t * (3.0 - 2.0 * t);
}

constexpr uint8_t r_offset = 1, g_offset = 0, b_offset = 2, w_offset = 3;

static inline uint32_t fade_color(uint32_t the_color, float the_fade_value)
{
    float val = clamp<float>(the_fade_value, 0.f, 1.f);

    uint8_t *ptr = (uint8_t*) &the_color;
    return  (uint32_t)(ptr[w_offset] * val) << 24 |
            (uint32_t)(ptr[b_offset] * val) << 16 |
            (uint32_t)(ptr[r_offset] * val) << 8 |
            (uint32_t)(ptr[g_offset] * val);
}

static inline uint32_t color_mix(uint32_t lhs, uint32_t rhs, float ratio)
{
    uint8_t *ptr_lhs = (uint8_t*) &lhs, *ptr_rhs = (uint8_t*) &rhs;

    return  (uint32_t)mix<float>(ptr_lhs[w_offset], ptr_rhs[w_offset], ratio) << 24 |
            (uint32_t)mix<float>(ptr_lhs[b_offset], ptr_rhs[b_offset], ratio) << 16 |
            (uint32_t)mix<float>(ptr_lhs[r_offset], ptr_rhs[r_offset], ratio) << 8 |
            (uint32_t)mix<float>(ptr_lhs[g_offset], ptr_rhs[g_offset], ratio);
}

static inline uint32_t color_add(uint32_t lhs, uint32_t rhs)
{
    uint8_t *ptr_lhs = (uint8_t*) &lhs, *ptr_rhs = (uint8_t*) &rhs;
    return  min((uint32_t)ptr_lhs[w_offset] + (uint32_t)ptr_rhs[w_offset], 255) << 24 |
            min((uint32_t)ptr_lhs[b_offset] + (uint32_t)ptr_rhs[b_offset], 255) << 16 |
            min((uint32_t)ptr_lhs[r_offset] + (uint32_t)ptr_rhs[r_offset], 255) << 8 |
            min((uint32_t)ptr_lhs[g_offset] + (uint32_t)ptr_rhs[g_offset], 255);
}

static inline void print_color(uint32_t the_color)
{
    char buf[32];
    uint8_t *ptr = (uint8_t*) &the_color;
    sprintf(buf, "R: %d - G: %d - B: %d - W: %d\n", ptr[r_offset], ptr[g_offset],
            ptr[b_offset], ptr[w_offset]);
    Serial.write(buf);
}

#define PI 3.1415926535f
#define PI_2 6.283185307f

class FastSinus
{
public:
    FastSinus()
    {
        float step = PI_2 / (float)m_array_size;
        float val = 0;

        for(uint32_t i = 0; i < m_array_size; ++i)
        {
            m_sin_table[i] = sin(val);
            val += step;
        }
    }

    inline float operator()(float the_val)
    {
        int index = (int)(fmodf(the_val, PI_2) / (PI_2) * m_array_size);
        index += index < 0 ? m_array_size : 0;
        return m_sin_table[index];
    }
private:
    static constexpr uint32_t m_array_size = 500;
    float m_sin_table[m_array_size];
};
