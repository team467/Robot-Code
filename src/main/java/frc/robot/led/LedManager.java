package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.effects.BreathingDownEffect;
import frc.robot.led.effects.BreathingUpEffect;
import frc.robot.led.effects.ColorCycleEffect;
import frc.robot.led.effects.Effect;
import frc.robot.led.effects.StaticEffect;

// #define LED_COUNT 10
// #define SECTION_COUNT 1
// #define PIN_TX 0

// uint8_t LED_RGB_BUFFER[LED_COUNT * 3] = {0x00};
// uint8_t LED_RGB_OUTPUT_BUFFER[LED_COUNT * 3] = {0x00};
// uint8_t LED_EFFECT_BUFFER[LED_COUNT * 3] = {0x00};
// uint8_t LED_BRIGHTNESS_BUFFER[LED_COUNT] = {0xFF};
// uint8_t SECTION_BUFFER[SECTION_COUNT * 2] = {0x00, 0x07};
public class LedManager {
    AddressableLEDBuffer outputBuffer

private static Effect[] effects = {
    new StaticEffect(),
    new BreathingUpEffect(),
    new BreathingDownEffect(),
    new ColorCycleEffect(),
};

public enum Effects {
    STATIC,
    BREATHING_IN,
    BREATHING_DOWN,
    COLOR_CYCLE
};

public void fillLedsBaseColorA(int startLed, int endLed, int value, Color color) {
    for (int i = startLed; i <= endLed; i++) {

    }
}

    public void fillLedsBaseColorA(int startLed, int endLed, int value, Color color) {
        for (int i = start_led; i <= end_led; ++i) {
            switch (value) {
                case id_led_base_color: {
                    LED_RGB_BUFFER[(i * 3) + 0] = data[0];
                    LED_RGB_BUFFER[(i * 3) + 1] = data[1];
                    LED_RGB_BUFFER[(i * 3) + 2] = data[2];
                    break;
                }

                case id_led_effect: {
                    LED_EFFECT_BUFFER[(i * 3) + 0] = data[0];
                    LED_EFFECT_BUFFER[(i * 3) + 1] = 0x00;
                    break;
                }

                case id_led_effect_spaced: {
                    LED_EFFECT_BUFFER[(i * 3) + 0] = data[0];
                    uint offset = ((i - start_led) * 0xFF);
                    offset /= (end_led - start_led) + 1;
                    LED_EFFECT_BUFFER[(i * 3) + 1] = (uint8_t) offset;
                    break;
                }

                case id_led_offset: {
                    LED_EFFECT_BUFFER[(i * 3) + 1] = data[0];
                    break;
                }

                case id_led_speed: {
                    LED_EFFECT_BUFFER[(i * 3) + 2] = data[0];
                    break;
                }

                case id_led_brightness: {
                    LED_BRIGHTNESS_BUFFER[i] = data[0];
                    break;
                }

                default: {
                    return 0;
                }
            }
        }
        return 1;
    }

uint8_t ws2812_fill_section(uint8_t section_id, uint8_t value, uint8_t *data) {
    return ws2812_fill_leds(SECTION_BUFFER[(section_id * 2) + 0], SECTION_BUFFER[(section_id * 2) + 1], value, data);
}

    // --------------------------------------------------------------------+
    // WS2812 UPDATE TASK
    // --------------------------------------------------------------------+
void ws2812_init(void)
{
    //set_sys_clock_48();
    stdio_init_all();
    puts("WS2812 Smoke Test");

    // todo get free sm
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, PIN_TX, 800000, false);

    for (int i = 0; i < LED_COUNT; ++i) {
        LED_RGB_BUFFER[(i * 3) + 0] = 0x00;
        LED_RGB_BUFFER[(i * 3) + 1] = 0x00;
        LED_RGB_BUFFER[(i * 3) + 2] = 0x00;

        LED_RGB_OUTPUT_BUFFER[(i * 3) + 0] = 0x00;
        LED_RGB_OUTPUT_BUFFER[(i * 3) + 1] = 0x00;
        LED_RGB_OUTPUT_BUFFER[(i * 3) + 2] = 0x00;

        LED_EFFECT_BUFFER[(i * 3) + 0] = 0x00;
        LED_EFFECT_BUFFER[(i * 3) + 1] = 0x00;
        LED_EFFECT_BUFFER[(i * 3) + 2] = 0x00;

        LED_BRIGHTNESS_BUFFER[i] = 0xFF;
    }
}

void led_effect_update_task(unsigned int t)
{
    for (int i = 0; i < LED_COUNT; ++i) {
        effect_table[LED_EFFECT_BUFFER[(i * 3) + 0]].eff(i, t);
    }
}

void ws2812_update_task(void)
{
    for (int i = 0; i < LED_COUNT; ++i) {
        unsigned int r = LED_RGB_OUTPUT_BUFFER[(i * 3) + 0];
        unsigned int g = LED_RGB_OUTPUT_BUFFER[(i * 3) + 1];
        unsigned int b = LED_RGB_OUTPUT_BUFFER[(i * 3) + 2];

        r *= LED_BRIGHTNESS_BUFFER[i];
        g *= LED_BRIGHTNESS_BUFFER[i];
        b *= LED_BRIGHTNESS_BUFFER[i];

        r /= 0xFF;
        g /= 0xFF;
        b /= 0xFF;

        put_pixel(urgb_u32(r, g, b));
    }
}
}