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

// serialization
#include "NodeTypes.h"

// state
#include "smart_bulb_state.h"

const uint8_t g_lora_address = 99;

// update rate in Hz
#define UPDATE_RATE 30
#define SERIAL_BUFSIZE 128

// analog-digital-converter (ADC)
#define ADC_BITS 10
constexpr uint32_t ADC_MAX = (1 << ADC_BITS) - 1U;

char g_serial_buf[SERIAL_BUFSIZE];
uint32_t g_buf_index = 0;

// helper variables for time measurement
long g_last_time_stamp = 0;
uint32_t g_time_accum = 0;

// update interval in millis
const int g_update_interval = 1000 / UPDATE_RATE;

enum TimerEnum
{
    TIMER_LED_OFF = 0,
    TIMER_BRIGHTNESS_MEASURE,
    TIMER_BATTERY_MEASURE,
    TIMER_LORA_SEND,
    TIMER_BUTTON_DEBOUNCE,
    TIMER_BRIGHTNESS_DEBOUNCE,
    NUM_TIMERS
};

// an array of Timer objects
kinski::Timer g_timer[NUM_TIMERS];

// helper for flashing PIN 13 (red onboard LED)
// to indicate update frequency
bool g_indicator = false;

bool g_use_indicator = false;

// there's a button
constexpr uint8_t g_button_pin = A2;
volatile bool g_button_pressed = false;

//! define our run-modes here
enum RunMode
{
    MODE_DEBUG = 1 << 0,
    MODE_RUNNING = 1 << 1
};
uint32_t g_run_mode = MODE_RUNNING;

bool g_leds_enabled = true;
constexpr uint8_t g_num_paths = 1;
constexpr uint8_t g_path_lengths[g_num_paths] = {1};
const uint8_t g_led_pins[g_num_paths] = {5};

LED_Path* g_path[g_num_paths];
ModeHelper *g_mode_sinus = nullptr, *g_mode_current = nullptr;
Mode_ONE_COLOR *g_mode_colour = nullptr;
CompositeMode *g_mode_composite = nullptr;

// timeout before leds turn off
float g_led_timeout = 4 * 3600.f;// 4h

// brightness measuring
constexpr uint8_t g_photo_pin = A3;

constexpr uint16_t g_photo_min = 160;
constexpr uint16_t g_photo_max = ADC_MAX;
constexpr uint16_t g_photo_thresh = 250;
uint16_t g_photo_val = 0;

// battery
#if defined(ARDUINO_SAMD_ZERO)
constexpr uint8_t g_battery_pin = A7;
#elif defined(ARDUINO_FEATHER_M4)
constexpr uint8_t g_battery_pin = A6;
#endif
uint16_t g_battery_val = 0;

// acceleration measuring
Adafruit_LIS3DH g_accel;
constexpr lis3dh_range_t g_sensor_range = LIS3DH_RANGE_4_G;
sensors_event_t g_sensor_event;
const uint16_t g_sense_interval = 2;
float g_last_accel_val = 0;
float g_max_accel_val = 0;
float g_accel_thresh = 1.f;

// median filtering for accelerometer
const uint16_t g_num_samples = 3;
RunningMedian g_running_median = RunningMedian(g_num_samples);

// lora assets
lora::config_t g_lora_config = {};

// bundle radio-driver, datagram-manager and crypto assets
lora::driver_struct_t m_rfm95 = {};

//! lora message buffer
uint8_t g_lora_buffer[RH_RF95_MAX_MESSAGE_LEN];

float g_lora_send_interval = 5.f;

// a simple state machine
State g_current_state = State::DAY_OFF;

// function declarations
void blink_status_led();

void set_address(uint8_t address)
{
    g_lora_config.address = address;

    // init RFM95 module
    if(lora::setup(g_lora_config, m_rfm95))
    {
        Serial.print("LoRa radio init complete -> now listening on adress: 0x");
        Serial.println(g_lora_config.address, HEX);
    }
    else
    {
       Serial.println("LoRa radio init failed");
       while(true){ blink_status_led(); }
    }
}

void lora_receive()
{
    uint8_t len = sizeof(g_lora_buffer);
    uint8_t from, to, msg_id, flags;

    // check for messages addressed to this node
    if(m_rfm95.manager->recvfrom(g_lora_buffer, &len, &from, &to, &msg_id, &flags))
    {
        // null-terminated
        // if(len < sizeof(g_lora_buffer)){ g_lora_buffer[len] = 0; }

        // if(from == m_node_info.server_address && to == g_lora_config.address)
        if(len >= sizeof(smart_bulb_t) && g_lora_buffer[0] == STRUCT_TYPE_SMART_BULB)
        {
            // auto rssi = m_rfm95.driver->lastRssi();
            // sprintf(g_serial_buf, "src: %d -- rssi: %d\n", from, rssi);
            // Serial.write(g_serial_buf);
            //
            // smart_bulb_t data = {};
            // memcpy(&data, g_lora_buffer, len);

            // sprintf(g_serial_buf, "{\n\tlight:%d\n\tacceleration: %d\n\tbattery: %d\n}\n",
            //         data.light_sensor, data.acceleration, data.battery);
            // Serial.write(g_serial_buf);
        }
    }
}

template<typename T> bool lora_send_status(const T &data)
{
    // data + checksum
    constexpr size_t num_bytes = sizeof(T) + 1;

    uint8_t crc_data[3 + sizeof(T)];
    crc_data[0] = g_lora_config.address;
    crc_data[1] = RH_BROADCAST_ADDRESS;

    memcpy(crc_data + 2, &data, sizeof(T));
    crc_data[sizeof(crc_data) - 1] = crc8(crc_data, 2 + sizeof(T));

    // send a broadcast-message
    return m_rfm95.manager->sendto(crc_data + 2, num_bytes, RH_BROADCAST_ADDRESS);
}

void blink_status_led()
{
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(13, HIGH);
    delay(500);
}

// seems borked since lora also uses interrupts!?
void button_interrupt()
{
    g_button_pressed = !digitalRead(g_button_pin);
}

void enable_leds(bool use_leds)
{
    for(uint8_t i = 0; i < g_num_paths; ++i)
    {
        for(uint8_t j = 0; j < g_path[i]->num_segments(); ++j)
        {
            Segment *s = g_path[i]->segment(j);
            s->set_active(use_leds);
        }
    }
    digitalWrite(13, use_leds);
}

void setup()
{
    analogReadResolution(ADC_BITS);

    // button
    pinMode(13, OUTPUT);
    pinMode(g_button_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(g_button_pin), button_interrupt, FALLING);

    // while(!Serial){ blink_status_led(); }
    Serial.begin(115200);

    srand(analogRead(A0));

    // enable photo sense pin
    pinMode(g_photo_pin, INPUT);

    // setup accelerometer
    g_accel = Adafruit_LIS3DH();
    if(!g_accel.begin())
    {
       Serial.print("could not connect accelerometer ...");
       while(true){ blink_status_led(); }
    }
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
    g_timer[TIMER_LED_OFF].set_callback([]()
    {
         g_current_state = next_state(g_current_state, Event::TIMER);
    });
    g_timer[TIMER_LED_OFF].expires_from_now(g_led_timeout);

    // start with current state
    apply_state(g_current_state);

    // brightness measuring
    g_timer[TIMER_BRIGHTNESS_MEASURE].set_callback([]()
    {
        g_photo_val = analogRead(g_photo_pin);

        // light sensor
        auto photo_event = g_photo_val > g_photo_thresh ? Event::SENSOR_HIGH : Event::SENSOR_LOW;
        g_current_state = next_state(g_current_state, photo_event);
    });
    g_timer[TIMER_BRIGHTNESS_MEASURE].set_periodic();
    g_timer[TIMER_BRIGHTNESS_MEASURE].expires_from_now(.2f);

    // battery measuring
    g_timer[TIMER_BATTERY_MEASURE].set_callback([]()
    {
        // voltage is divided by 2, so multiply back
        constexpr float voltage_divider = 2.f;
        auto raw_bat_measure = analogRead(g_battery_pin);

        float voltage = 3.3f * (float)raw_bat_measure * voltage_divider / (float)ADC_MAX;
        g_battery_val = static_cast<uint8_t>(map_value<float>(voltage, 3.6f, 4.2f, 0.f, 255.f));
        Serial.printf("battery: %d%%\n", 100 * g_battery_val / 255);
    });
    g_timer[TIMER_BATTERY_MEASURE].set_periodic();
    g_timer[TIMER_BATTERY_MEASURE].expires_from_now(10.f);

    // lora config
    set_address(g_lora_address);

    g_timer[TIMER_LORA_SEND].set_callback([]()
    {
        smart_bulb_t smart_bulb = {};
        smart_bulb.leds_enabled = static_cast<uint8_t>(g_current_state);
        smart_bulb.battery = g_battery_val;
        smart_bulb.acceleration = static_cast<uint8_t>(map_value<float>(g_max_accel_val, 0.f, 4.f, 0, 255));
        smart_bulb.light_sensor = static_cast<uint8_t>(map_value<float>(g_photo_val, g_photo_min, g_photo_max, 0, 255));
        g_max_accel_val = 0;

        lora_send_status(smart_bulb);
    });
    g_timer[TIMER_LORA_SEND].set_periodic();
    g_timer[TIMER_LORA_SEND].expires_from_now(g_lora_send_interval);
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
    float g_factor = sqrtf(g_running_median.getMedian()) / base_g;
    g_last_accel_val = max(g_last_accel_val, max(g_factor - 1.f, 0.f));
    g_max_accel_val = max(g_last_accel_val, g_max_accel_val);

    // poll Timer objects
    for(uint32_t i = 0; i < NUM_TIMERS; ++i){ g_timer[i].poll(); }

    // receive
    lora_receive();
    
    if(g_button_pressed && !digitalRead(g_button_pin))
    {
        g_button_pressed = false;
        static uint32_t i = 0;
        Serial.print("button pressed: "); Serial.println(i++);

        g_current_state = next_state(g_current_state, Event::BUTTON);
    }
    // check for "shake" event
    else if(g_last_accel_val > g_accel_thresh)
    {
        g_current_state = next_state(g_current_state, Event::SHAKE);
    }

    if(g_time_accum >= g_update_interval)
    {
        enable_leds(g_leds_enabled);

        // TODO: random palette
        // if(rand() / (float)RAND_MAX > 0.5f){}

        auto color = color_mix(ORANGE, AQUA, clamp(g_last_accel_val / 2.f, 0.f, 1.f));
        g_mode_colour->set_color(color);

        for(uint8_t i = 0; i < g_num_paths; ++i)
        {
            g_mode_current->process(g_path[i], g_time_accum);
            g_path[i]->update(g_time_accum);
        }

        // clear acceleration max
        g_last_accel_val = 0;

        // clear time accumulator
        g_time_accum = 0;
    }
    yield();
}

void apply_state(State current_state)
{
    switch (current_state)
    {
    case State::DAY_ON:
        g_leds_enabled = true;
        break;

    case State::DAY_OFF:
        g_leds_enabled = false;
        break;

    case State::NIGHT_ON:

        // enable leds, start timer
        g_leds_enabled = true;
        g_timer[TIMER_LED_OFF].expires_from_now(g_led_timeout);
        break;

    case State::NIGHT_OFF:
        g_leds_enabled = false;
        break;

    default:
        break;
    }
}

State next_state(State current_state, Event event)
{
    State state = current_state;
    
    // luma debounce
    bool debounceLuma = !g_timer[TIMER_BRIGHTNESS_DEBOUNCE].has_expired();
    bool debounceButton = !g_timer[TIMER_BUTTON_DEBOUNCE].has_expired();

    switch (current_state)
    {
    case State::DAY_OFF:
    if((!debounceButton && event == Event::BUTTON) || event == Event::SHAKE){ state = State::DAY_ON; }
    else if(!debounceLuma && event == Event::SENSOR_LOW){ state = State::NIGHT_ON; }
        break;
    
    case State::DAY_ON:
    if((!debounceButton && event == Event::BUTTON)){ state = State::DAY_OFF; }
    else if(!debounceLuma && event == Event::SENSOR_LOW){ state = State::NIGHT_ON; }
        break;

    case State::NIGHT_OFF:
    if((!debounceButton && event == Event::BUTTON) || event == Event::SHAKE){ state = State::NIGHT_ON; }
    else if(!debounceLuma && event == Event::SENSOR_HIGH){ state = State::DAY_OFF; }
        break;

    case State::NIGHT_ON:
    if((!debounceButton && event == Event::BUTTON) || event == Event::TIMER){ state = State::NIGHT_OFF; }
    else if(!debounceLuma && event == Event::SENSOR_HIGH){ state = State::DAY_OFF; }
        break;

    default:
        break;
    }

    // apply changes if necessary
    if(state != current_state)
    {
        apply_state(state);

        if(event == Event::SENSOR_LOW || event == Event::SENSOR_HIGH)
        {
            // set debounce timer in s
            constexpr float luma_timeout = 60.f;
            g_timer[TIMER_BRIGHTNESS_DEBOUNCE].expires_from_now(luma_timeout);
        }
        else if(event == Event::BUTTON)
        {
            // set debounce timer in s
            constexpr float debounce_timeout = .25f;
            g_timer[TIMER_BUTTON_DEBOUNCE].expires_from_now(debounce_timeout);
        }
    }
    return state;
}