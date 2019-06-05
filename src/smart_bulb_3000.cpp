#include "Arduino.h"
#include "ModeHelpers.h"
#include "Timer.hpp"
#include "device_id.h"

// #include <Scheduler.h>

// accelerometer
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "RunningMedian.h"
#include "utils.h"
#include "vec3.h"

// lora / radiohead
#include "rfm95_config.h"

// update rate in Hz
#define UPDATE_RATE 30
#define SERIAL_BUFSIZE 128

char g_serial_buf[SERIAL_BUFSIZE];
uint32_t g_buf_index = 0;

// helper variables for time measurement
long g_last_time_stamp = 0;
uint32_t g_time_accum = 0;

// update interval in millis
const int g_update_interval = 1000 / UPDATE_RATE;

// an array of Timer objects is provided
constexpr uint32_t g_num_timers = 2;
enum TimerEnum
{
    TIMER_RUNMODE = 0,
    TIMER_BRIGHTNESS_MEASURE = 1
};
kinski::Timer g_timer[g_num_timers];

// helper for flashing PIN 13 (red onboard LED)
// to indicate update frequency
bool g_indicator = false;

bool g_use_indicator = false;

//! define our run-modes here
enum RunMode
{
    MODE_DEBUG = 1 << 0,
    MODE_RUNNING = 1 << 1,
    MODE_STREAMING = 1 << 2
};
uint32_t g_run_mode = MODE_RUNNING;

constexpr uint8_t g_num_paths = 1;
constexpr uint8_t g_path_lengths[] = {1};
const uint8_t g_led_pins[] = {5};

LED_Path* g_path[g_num_paths];
ModeHelper *g_mode_sinus = nullptr, *g_mode_current = nullptr;
Mode_ONE_COLOR *g_mode_colour = nullptr;
CompositeMode *g_mode_composite = nullptr;

// brightness measuring
constexpr uint8_t g_photo_pin = A5;
constexpr uint16_t g_photo_thresh = 60;

// acceleration measuring
Adafruit_LIS3DH g_accel;
constexpr lis3dh_range_t g_sensor_range = LIS3DH_RANGE_4_G;
sensors_event_t g_sensor_event;
const uint16_t g_sense_interval = 5;
float g_last_accel_val = 0;

// median filtering for accelerometer
const uint16_t g_num_samples = 5;
RunningMedian g_running_median = RunningMedian(g_num_samples);

// lora assets
lora::config_t g_lora_config = {};

// bundle radio-driver, datagram-manager and crypto assets
lora::driver_struct_t m_rfm95 = {};

void set_address(uint8_t address)
{
    g_lora_config.address = address;

    // init RFM95 module
    if(lora::setup(g_lora_config, m_rfm95))
    {
        Serial.print("LoRa radio init complete -> now listening on adress: 0x");
        Serial.println(g_lora_config.address, HEX);
    }
    else{ Serial.println("LoRa radio init failed"); }
}

void blink()
{
    digitalWrite(13, true);
    delay(666);
    digitalWrite(13, false);
    delay(666);
}

void setup()
{
    // while(!Serial){ delay(10); }
    Serial.begin(2000000);

    srand(analogRead(A0));

    // drives our status LED
    pinMode(13, OUTPUT);
    digitalWrite(13, 0);

    // enable photo sense pin
    pinMode(g_photo_pin, INPUT);

    //setup accelerometer
    g_accel = Adafruit_LIS3DH();
    if(!g_accel.begin()){ Serial.print("could not connect accelerometer ..."); }
    g_accel.setRange(g_sensor_range);

    // init path objects with pin array
    for(uint8_t i = 0; i < g_num_paths; ++i)
    {
         g_path[i] = new LED_Path(g_led_pins[i], g_path_lengths[i]);
    }

    // create ModeHelper objects
    g_mode_colour = new Mode_ONE_COLOR();
    g_mode_sinus = new SinusFill();
    g_mode_current = g_mode_composite = new CompositeMode();
    g_mode_composite->add_mode(g_mode_colour);
    g_mode_composite->add_mode(g_mode_sinus);

    // timer callback to reset the runmode after streaming
    g_timer[TIMER_RUNMODE].set_callback([](){ g_run_mode = MODE_RUNNING; });

    // brightness measuring
    g_timer[TIMER_BRIGHTNESS_MEASURE].set_callback([]()
    {
        auto val = analogRead(g_photo_pin);
        bool use_leds = val < g_photo_thresh;

        for(uint8_t i = 0; i < g_num_paths; ++i)
        {
            for(uint8_t j = 0; j < g_path[i]->num_segments(); ++j)
            {
                Segment *s = g_path[i]->segment(j);
                s->set_active(use_leds);
            }
        }
    });
    g_timer[TIMER_BRIGHTNESS_MEASURE].set_periodic();
    g_timer[TIMER_BRIGHTNESS_MEASURE].expires_from_now(.2f);

    // Scheduler.start(blink);

    // lora config
    set_address(69);
}

void loop()
{
    // time measurement
    uint32_t delta_time = millis() - g_last_time_stamp;
    g_last_time_stamp = millis();
    g_time_accum += delta_time;

    // measure acceleration
    for(uint8_t i = 0; i < g_num_samples; i++)
    {
        // sensor reading
        g_accel.getEvent(&g_sensor_event);
        vec3 measurement(g_sensor_event.acceleration.v);
        g_running_median.add(measurement.length2());

        // wait the desired interval
        delay(g_sense_interval);
    }
    constexpr float base_g =  9.82f;
    float g_factor = sqrt(g_running_median.getMedian()) / base_g;
    g_last_accel_val = max(g_last_accel_val, max(g_factor - 1.f, 0.f));

    // poll Timer objects
    for(uint32_t i = 0; i < g_num_timers; ++i){ g_timer[i].poll(); }

    if(g_time_accum >= g_update_interval)
    {
        // flash red indicator LED
        if(g_use_indicator){ digitalWrite(13, g_indicator); }
        g_indicator = !g_indicator;

        if(!(g_run_mode & MODE_STREAMING))
        {
            auto color = color_mix(ORANGE, AQUA, clamp(g_last_accel_val / 2.f, 0.f, 1.f));
            g_mode_colour->set_color(color);

            // Serial.println(g_last_accel_val);

            for(uint8_t i = 0; i < g_num_paths; ++i)
            {
                g_mode_current->process(g_path[i], g_time_accum);
                g_path[i]->update(g_time_accum);
            }
        }

        // clear acceleration max
        g_last_accel_val = 0;

        // clear time accumulator
        g_time_accum = 0;
    }
    yield();
}
