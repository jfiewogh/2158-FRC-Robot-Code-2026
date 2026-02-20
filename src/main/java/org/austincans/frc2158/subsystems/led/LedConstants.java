package org.austincans.frc2158.subsystems.led;

import java.io.ObjectInputFilter.Config;

import org.austincans.frc2158.subsystems.led.LedIOBlinkin.BlinkinPattern;
import org.austincans.frc2158.util.ConfigUtil;

public class LedConstants {
    public static final int PWMPort = ConfigUtil.getConfigInt("PWM_LED_PORT", 0);
    public static final int numOfLeds = ConfigUtil.getConfigInt("NUM_OF_LEDS", 60);
    public static final int blinkinID = ConfigUtil.getConfigInt("BLINKIN_LED_CONTROLLER_PORT", 0);
}
