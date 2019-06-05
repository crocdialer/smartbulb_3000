#include "LED_Path.h"

Segment::Segment(uint8_t *the_data, uint32_t the_length):
m_data(the_data),
m_length(the_length)
{

}

LED_Path::LED_Path(uint32_t the_pin, uint32_t the_num_segments):
m_num_segments(the_num_segments)
{
    m_strip = new LedType(SEGMENT_LENGTH * the_num_segments, the_pin, CURRENT_LED_TYPE);
    m_strip->begin();

    // we're scaling the brightness ourselves
    m_strip->setBrightness(255);

    // initialize all pixels to 0
    m_strip->show();

    m_data = (uint8_t*)m_strip->getPixels();
    m_current_max = num_leds();

    m_segments = new Segment*[the_num_segments];

    for(uint32_t i = 0; i < the_num_segments; ++i)
    {
        m_segments[i] = new Segment(m_data + i * SEGMENT_LENGTH * BYTES_PER_PIXEL,
                                    SEGMENT_LENGTH);
    }
}

LED_Path::~LED_Path()
{
    if(m_strip){ delete m_strip; }
    if(m_segments)
    {
        for(uint32_t i = 0; i < m_num_segments; ++i){ delete m_segments[i]; }
        delete[] m_segments;
    }
}

Segment* LED_Path::segment(uint32_t the_index) const
{
    if(the_index < m_num_segments)
        return m_segments[the_index];
    return nullptr;
};

void LED_Path::clear()
{
    memset(m_data, 0, num_leds() * BYTES_PER_PIXEL);
}

void LED_Path::update(uint32_t the_delta_time)
{
    // show!?
    m_strip->show();
}

void LED_Path::set_brightness(float the_brightness)
{
     m_brightness = the_brightness;
     // m_strip->setBrightness(255 * m_brightness);
}

void LED_Path::set_all_segments(uint32_t the_color)
{
    for(uint32_t i = 0; i < m_num_segments; ++i)
    {
        m_segments[i]->set_color(the_color);
    }
}
