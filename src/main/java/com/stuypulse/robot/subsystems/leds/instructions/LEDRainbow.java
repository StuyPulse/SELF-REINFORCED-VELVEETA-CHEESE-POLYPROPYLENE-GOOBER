package com.stuypulse.robot.subsystems.leds.instructions;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDRainbow implements LEDInstruction {
    private int m_rainbowFirstPixelHue = 0;

    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        for (int i = 0; i < ledsBuffer.getLength(); i++) {
            final int hue = (m_rainbowFirstPixelHue + (i * 180 / ledsBuffer.getLength())) % 180;
            ledsBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
    }
}
