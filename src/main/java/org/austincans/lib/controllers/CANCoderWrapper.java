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

package org.austincans.lib.controllers;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class CANCoderWrapper {
  private CANcoder encoder;
  private boolean inverted = false;

  public CANCoderWrapper() {
    super();
  }

  public CANCoderWrapper(int canID) {
    this(canID, false, 0);
  }

  public CANCoderWrapper(int canID, boolean inverted, double offsetRotations) {
    super();
    this.encoder = new CANcoder(canID);
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset = offsetRotations;
    canCoderConfig.MagnetSensor.SensorDirection =
        inverted
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(canCoderConfig);
  }

  public double getAbsPositionRadians() {
    double absPosition = encoder.getAbsolutePosition().getValueAsDouble();
    if (inverted) {
      absPosition = 1 - absPosition;
    }
    absPosition = absPosition % 1;
    return rotationsToRadians(absPosition) % (Math.PI * 2);
  }

  private double rotationsToRadians(double rotations) {
    return 2 * Math.PI * rotations;
  }

  public double getVelocityRadians() {
    return rotationsToRadians(2 * Math.PI * encoder.getVelocity().getValueAsDouble());
  }

  public CANcoder getCanandcoder() {
    return this.encoder;
  }

  public void setCanandcoder(CANcoder encoder) {
    this.encoder = encoder;
  }
}
