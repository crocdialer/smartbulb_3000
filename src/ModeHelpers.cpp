#include "ModeHelpers.h"

ModeHelper::ModeHelper(){}

void ModeHelper::set_trigger_time(uint32_t the_min, uint32_t the_max)
{
    m_trigger_time_min = the_min;
    m_trigger_time_max = the_max;
}

///////////////////////////////////////////////////////////////////////////////

Mode_ONE_COLOR::Mode_ONE_COLOR():ModeHelper()
{

}

void Mode_ONE_COLOR::process(LED_Path* the_path, uint32_t the_delta_time)
{
    m_time_accum += the_delta_time;

    if(m_time_accum > m_trigger_time)
    {
        the_path->set_all_segments(m_next_color);

        // uint32_t col_index = clamp<uint32_t>(random<uint32_t>(0, g_num_colors),
        //                                      0, g_num_colors - 1);
        // m_next_color = g_colors[col_index];

        // Serial.println((int)(col_index));
        m_trigger_time = random<uint32_t>(m_trigger_time_min, m_trigger_time_max);
        m_time_accum = 0;
    }
}

void Mode_ONE_COLOR::reset(LED_Path* the_path)
{
    m_time_accum = m_trigger_time = 0;

    for(uint32_t i = 0; i < the_path->num_segments(); ++i)
    {
        the_path->segment(i)->set_active(true);
    }
}

///////////////////////////////////////////////////////////////////////////////

ModeFlash::ModeFlash():ModeHelper()
{
    set_trigger_time(8000, 15000);
}

void ModeFlash::process(LED_Path* the_path, uint32_t the_delta_time)
{
    m_time_accum += the_delta_time;

    if(m_time_accum > m_trigger_time)
    {
        the_path->set_current_max(0);
        set_flash_speed(random<uint32_t>(the_path->num_leds(), the_path->num_leds() * 4));//950, 5500
        m_trigger_time = random<uint32_t>(m_trigger_time_min, m_trigger_time_max);
        m_time_accum = 0;
    }

    auto new_max_index = min(the_path->num_leds(),
                             the_path->current_max() + m_flash_speed * the_delta_time / 1000.f);
    the_path->set_current_max(new_max_index);
}

void ModeFlash::reset(LED_Path* the_path)
{
    the_path->set_current_max(the_path->num_leds() - 1);
}

///////////////////////////////////////////////////////////////////////////////

Mode_Segments::Mode_Segments():ModeHelper()
{
    set_trigger_time(1200 , 6000);
}

void Mode_Segments::process(LED_Path* the_path, uint32_t the_delta_time)
{
    m_time_accum += the_delta_time;

    if(m_time_accum > m_trigger_time)
    {
        for(uint32_t i = 0; i < the_path->num_segments(); ++i)
        {
            the_path->segment(i)->set_active(random<float>(0, 1) > .5f);

            uint32_t col_index = clamp<uint32_t>(random<uint32_t>(0, g_num_colors),
                                                 0, g_num_colors - 1);
            the_path->segment(i)->set_color(g_colors[col_index]);
        }
        m_trigger_time = random<uint32_t>(m_trigger_time_min, m_trigger_time_max);
        m_time_accum = 0;
    }
}

void Mode_Segments::reset(LED_Path* the_path)
{
    m_time_accum = m_trigger_time = 0;

    for(uint32_t i = 0; i < the_path->num_segments(); ++i)
    {
        the_path->segment(i)->set_active(true);
    }
}

///////////////////////////////////////////////////////////////////////////////

FastSinus SinusFill::s_fast_sin;

SinusFill::SinusFill():ModeHelper()
{
    set_sinus_offsets(random<int>(0, 256), random<int>(0, 256));
}

void SinusFill::process(LED_Path* the_path, uint32_t the_delta_time)
{
    the_path->clear();
    auto max_index = the_path->current_max();
    float brightness = the_path->brightness();

    for(uint32_t i = 0; i < the_path->num_segments(); ++i)
    {
        Segment *seg = the_path->segment(i);

        if(!seg->active()){ continue; }
        uint32_t c = seg->color();
        swap(((uint8_t*) &c)[0], ((uint8_t*) &c)[1]);

        uint8_t *ptr = seg->data();
        uint8_t *end_ptr = ptr + seg->length() * BYTES_PER_PIXEL;

        for(;ptr < end_ptr; ptr += BYTES_PER_PIXEL)
        {
            uint32_t current_index = (ptr - the_path->data()) / BYTES_PER_PIXEL;
            if(current_index >= max_index){ goto finished; }

            float sin_val = create_sinus_val(current_index);
            uint32_t fade_col = fade_color(c, brightness * sin_val);
            memcpy(ptr, &fade_col, BYTES_PER_PIXEL);
        }
    }

finished:

    // m_strip->show();
    // m_current_max = min(num_leds(), m_current_max + m_flash_speed * the_delta_time / 1000.f);

    // advance sinus offsets
    for(uint32_t i = 0; i < 2; ++i)
    {
        m_sinus_offsets[i] += m_sinus_speeds[i] * (the_delta_time / 1000.f);
    }
}

void SinusFill::reset(LED_Path* the_path)
{

}

///////////////////////////////////////////////////////////////////////////////

CompositeMode::CompositeMode():ModeHelper()
{
    set_trigger_time(1000 * 30 , 1000 * 120);
    memset(m_mode_helpers, 0, sizeof(m_mode_helpers));

    // m_mode_helpers[0] = new Mode_ONE_COLOR(the_path);
    // m_mode_helpers[1] = new SinusFill(the_path);
    // m_mode_helpers[2] = new ModeFlash(the_path);
}

void CompositeMode::process(LED_Path* the_path, uint32_t the_delta_time)
{
    m_time_accum += the_delta_time;

    // process child modes
    for(int i = 0; i < s_max_num_modes; ++i)
    {
         if(m_mode_helpers[i]){ m_mode_helpers[i]->process(the_path, the_delta_time); }
    }

    if(m_time_accum > m_trigger_time)
    {
        // // change mode here
        // if(m_mode_helpers[2])
        // {
        //     m_mode_helpers[1]->reset();
        //     swap(m_mode_helpers[1], m_mode_helpers[2]);
        //     m_shorter_duration = !m_shorter_duration;
        // }
        //
        // m_trigger_time = random<uint32_t>(m_trigger_time_min, m_trigger_time_max);
        // if(m_shorter_duration){ m_trigger_time /= 3; }

        m_time_accum = 0;
    }
};

void CompositeMode::reset(LED_Path* the_path)
{
    m_time_accum = m_trigger_time = 0;
    for(int i = 0; i < s_max_num_modes; ++i)
    {
         if(m_mode_helpers[i]){ m_mode_helpers[i]->reset(the_path); }
    }
}

void CompositeMode::add_mode(ModeHelper *m)
{
    for(int i = 0; i < s_max_num_modes; ++i)
    {
         if(!m_mode_helpers[i])
         {
             m_mode_helpers[i] = m;
             m_num_modes++;
             break;
         }
    }
}

void CompositeMode::remove_mode(ModeHelper *m)
{
    for(int i = 0; i < s_max_num_modes; ++i)
    {
         if(m_mode_helpers[i] == m)
         {
             m_mode_helpers[i] = nullptr;
             m_num_modes--;
             break;
         }
    }
}
