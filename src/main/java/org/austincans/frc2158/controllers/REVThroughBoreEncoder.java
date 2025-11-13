// Copyright (c) 2025 the ausTIN CANs
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
// Updated on Wednesday November 12, 2025 at 18:18:23 CST

package org.austincans.frc2158.controllers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class REVThroughBoreEncoder {
  private DutyCycleEncoder m_dutyCycleEncoder;
  private Rotation2d m_offset = Rotation2d.fromDegrees(0);
  private boolean m_Inverted;

  // 2nd parameter is the range of values. This sensor will output between
  // 0 and the passed in value.
  // 3rd parameter is the the physical value where you want "0" to be. How
  // to measure this is fairly easy. Set the value to 0, place the mechanism
  // where you want "0" to be, and observe the value on the dashboard, That
  // is the value to enter for the 3rd parameter.
  public REVThroughBoreEncoder(int dioChannel, double fullRange, double expectedZero) {
    m_dutyCycleEncoder = new DutyCycleEncoder(dioChannel, fullRange, expectedZero);
    m_dutyCycleEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);

    // If you know the frequency of your sensor, uncomment the following
    // method, and set the method to the frequency of your sensor.
    // This will result in more stable readings from the sensor.
    // Do note that occasionally the datasheet cannot be trusted
    // and you should measure this value. You can do so with either
    // an oscilloscope, or by observing the "Frequency" output
    // on the dashboard while running this sample. If you find
    // the value jumping between the 2 values, enter halfway between
    // those values. This number doesn't have to be perfect,
    // just having a fairly close value will make the output readings
    // much more stable.
    m_dutyCycleEncoder.setAssumedFrequency(967.8);
    m_dutyCycleEncoder.setInverted(false);
  }

  public REVThroughBoreEncoder(
      int dioChannel, boolean inverted, double offset, double fullRange, double expectedZero) {
    this(dioChannel, fullRange, expectedZero);
    setInverted(inverted);
    setOffset(Rotation2d.fromRadians(offset));
  }

  public Rotation2d getOffset() {
    return m_offset;
  }

  public void setOffset(Rotation2d m_offset) {
    this.m_offset = m_offset;
  }

  public boolean isInverted() {
    return m_Inverted;
  }

  public void setInverted(boolean inverted) {
    m_Inverted = inverted;
  }

  // Connected can be checked, and uses the frequency of the encoder
  public boolean isConnected() {
    return (m_dutyCycleEncoder.isConnected());
  }

  // Duty Cycle Frequency in Hz
  public int getFrequency() {
    return (m_dutyCycleEncoder.getFrequency());
  }

  public Rotation2d getAngle() {
    double angle = m_dutyCycleEncoder.get() % 360;

    angle -= m_offset.getDegrees();

    if (m_Inverted) {
      angle = 360 - angle;
    }

    if (angle < 0.0) {
      angle = angle + 360;
    }

    return Rotation2d.fromDegrees(angle % 360);
  }

  // By default, the output will wrap around to the full range value
  // when the sensor goes below 0. However, for moving mechanisms this
  // is not usually ideal, as if 0 is set to a hard stop, its still
  // possible for the sensor to move slightly past. If this happens
  // The sensor will assume its now at the furthest away position,
  // which control algorithms might not handle correctly. Therefore
  // it can be a good idea to slightly shift the output so the sensor
  // can go a bit negative before wrapping. Usually 10% or so is fine.
  // This does not change where "0" is, so no calibration numbers need
  // to be changed.
  public double getOutput() {
    return (m_dutyCycleEncoder.get());
  }
}
