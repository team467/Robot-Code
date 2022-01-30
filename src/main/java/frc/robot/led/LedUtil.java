package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LedUtil {
    private static final int[] ledGamma = { // Brightness ramp for LEDs
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
        2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6,
        6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 11,
        11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18,
        18, 19, 19, 20, 20, 21, 21, 22, 23, 23, 24, 24, 25, 26, 26, 27,
        28, 28, 29, 30, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39,
        40, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 49, 50, 51, 52, 53,
        54, 55, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 68, 69, 70, 71,
        72, 74, 75, 76, 77, 79, 80, 81, 83, 84, 85, 87, 88, 89, 91, 92,
        94, 95, 97, 98, 99, 101, 103, 104, 106, 107, 109, 110, 112, 113, 115, 117,
        118, 120, 122, 123, 125, 127, 129, 130, 132, 134, 136, 138, 139, 141, 143, 145,
        147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177,
        179, 182, 184, 186, 188, 190, 193, 195, 197, 200, 202, 204, 207, 209, 211, 214,
        216, 219, 221, 223, 226, 228, 231, 234, 236, 239, 241, 244, 247, 249, 252, 255};


    public static Color8Bit gammaCorrectColor(Color8Bit color) {
        return new Color8Bit(ledGamma[color.red], ledGamma[color.green], ledGamma[color.blue]);
    }
    
    public static Color gammaCorrectColor(Color color) {
        return new Color(gammaCorrectColor(new Color8Bit(color)));
    }

    public static Color brightnessCorrectColor(Color color, double brightness) {
        return new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
    }

    public static Color8Bit brightnessCorrectColor(Color8Bit color, double brightness) {
        return new Color8Bit(brightnessCorrectColor(new Color(color), brightness));
    }

    public static void setColor(AddressableLEDBuffer buffer, int led, Color color, double brightness) {
        buffer.setLED(led, LedUtil.gammaCorrectColor(LedUtil.brightnessCorrectColor(color, brightness)));
    }

    public static void setColor(AddressableLEDBuffer buffer, int led, Color8Bit color, double brightness) {
        buffer.setLED(led, LedUtil.gammaCorrectColor(LedUtil.brightnessCorrectColor(color, brightness)));
    }

    public static double hueToRGB(double p, double q, double t) {
        if (t < 0) t += 1;
        if (t > 1) t -= 1;
        if (t < (double) 1 / 6) return p + (q - p) * 6 * t;
        if (t < (double) 1 / 2) return q;
        if (t < (double) 2 / 3) return p + (q - p) * ((double) 2 / 3 - t) * 6;
        return p;
    }
    
    public static Color hslToRGB(double h, double s, double l) {
        double r, g, b;
    
        if (s == 0) {
            r = g = b = l; // achromatic
        } else {
            double q = (l < 0.5) ? l * (1 + s) : l + s - l * s;
            double p = 2 * l - q;
            r = hueToRGB(p, q, h + (double) 1 / 3);
            g = hueToRGB(p, q, h);
            b = hueToRGB(p, q, h - (double) 1 / 3);
        }
    
        return new Color(r, g, b);
    }
}
