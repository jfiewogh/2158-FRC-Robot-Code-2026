package org.austincans.frc2158.subsystems.led;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import org.austincans.frc2158.subsystems.led.LedIOBlinkin.BlinkinPattern;
import org.checkerframework.checker.units.qual.m;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

public class LedIOAddressableLed implements LedIO {
    public AddressableLED m_led = new AddressableLED(LedConstants.PWMPort);
    public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LedConstants.numOfLeds);

    public void LedIOAddressableLed() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    LEDPattern orange = LEDPattern.solid(Color.kOrange);
    LEDPattern gold = LEDPattern.solid(Color.kGold);
    LEDPattern black = LEDPattern.solid(Color.kBlack);
    LEDPattern red = LEDPattern.solid(Color.kRed);

    // Drive LED set up
    Map<Double, Color> maskSteps = Map.of(0.5, Color.kWhite, 0.5, Color.kBlack);
    LEDPattern drivingBase = LEDPattern.solid(Color.kBlue);
    LEDPattern drivingMask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(0.25));
    LEDPattern drivingPattern = drivingBase.mask(drivingMask);

    // Climbing LED set up
    Distance ledSpacing = Meters.of(1 / 120.0);
    LEDPattern climbingBase = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kGreen, Color.kLightGreen);
    LEDPattern climbingPattern = climbingBase.scrollAtRelativeSpeed(Percent.per(Second).of(25));
    LEDPattern climbingAbsolute = climbingBase.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);

    // Idle LED set up
    LEDPattern idleBase = LEDPattern.solid(Color.kYellow);
    LEDPattern ideleMask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(0.25));
    LEDPattern idlePattern = idleBase.mask(ideleMask);


    @Override
    public void updateInputs(LedIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void setPattern(LedState state) {
        switch (state) {
            case READYTOSHOOT -> {
                green.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            }
            case INTAKED -> {
                rainbow.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            }
            case DRIVING -> {
                drivingPattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            }
            case CLIMBING -> {
                climbingPattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            }
            case SHUTTLING -> {
                orange.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            }
            case ALIGNED -> {
                gold.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            }
            case NONE -> {
                black.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            }
            case IDLE -> {
                idlePattern.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            }
            case SHOOTING -> {
                red.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            }
            default -> {
                black.applyTo(m_ledBuffer);
                m_led.setData(m_ledBuffer);
            }
        }
    }

}