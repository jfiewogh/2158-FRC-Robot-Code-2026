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
// Updated on Thursday November 13, 2025 at 17:00:51 CST

package org.austincans.frc2158.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ElevatorIOSpark implements ElevatorIO {
  private int canID = 0;
  private final double maxVelocity = 1; // meters per second
  private final double maxAcceleration = 1; // meters per second squared
  private final double drumRadius = 0.0254; // meters
  private final double gearRatio = 0;
  private final boolean brakeMode = true;

  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private SparkFlexConfig config;

  @SuppressWarnings("unused")
  private final boolean enableStatorLimit = true;

  private final double statorCurrentLimit = 40;

  @SuppressWarnings("unused")
  private final boolean enableSupplyLimit = false;

  @SuppressWarnings("unused")
  private final double supplyCurrentLimit = 40;

  public ElevatorIOSpark() {
    motor = new SparkFlex(canID, MotorType.kBrushless);
    config = new SparkFlexConfig();
    config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Set current limits
    config.smartCurrentLimit((int) statorCurrentLimit);

    // Save configuration
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionMeters = encoder.getPosition();
    inputs.voltage = motor.getBusVoltage();
    inputs.current = motor.getOutputCurrent();
    inputs.velocityMetersPerSecond = encoder.getVelocity() * gearRatio;
    inputs.temperatureC = motor.getMotorTemperature();
  }
}
