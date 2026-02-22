// Copyright (c) 2026 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// oooooooooooo ooooooooo.     .oooooo.        ooooooooo.              .o8                     .   
// `888'     `8 `888   `Y88.  d8P'  `Y8b       `888   `Y88.           "888                   .o8   
//  888          888   .d88' 888                888   .d88'  .ooooo.   888oooo.   .ooooo.  .o888oo 
//  888oooo8     888ooo88P'  888                888ooo88P'  d88' `88b  d88' `88b d88' `88b   888   
//  888    "     888`88b.    888                888`88b.    888   888  888   888 888   888   888   
//  888          888  `88b.  `88b    ooo        888  `88b.  888   888  888   888 888   888   888 . 
// o888o        o888o  o888o  `Y8bood8P'       o888o  o888o `Y8bod8P'  `Y8bod8P' `Y8bod8P'   "888" 
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// Updated on Saturday February 21, 2026 at 20:04:14 CST

package org.austincans.frc2158.subsystems.led;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public class LedIOAddressableLed implements LedIO {
  public AddressableLED m_led = new AddressableLED(LedConstants.PWMPort);
  public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LedConstants.numOfLeds);
  LedState state = LedState.IDLE;

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
  LEDPattern drivingMask =
      LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(0.25));
  LEDPattern drivingPattern = drivingBase.mask(drivingMask);

  // Climbing LED set up
  Distance ledSpacing = Meters.of(1 / 120.0);
  LEDPattern climbingBase =
      LEDPattern.gradient(GradientType.kDiscontinuous, Color.kGreen, Color.kLightGreen);
  LEDPattern climbingPattern = climbingBase.scrollAtRelativeSpeed(Percent.per(Second).of(25));
  LEDPattern climbingAbsolute =
      climbingBase.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);

  // Idle LED set up
  LEDPattern idleBase = LEDPattern.solid(Color.kYellow);
  LEDPattern ideleMask =
      LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(0.25));
  LEDPattern idlePattern = idleBase.mask(ideleMask);

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.state = this.state;
  }

  @Override
  public void setPattern(LedState state) {
    this.state = state;
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
