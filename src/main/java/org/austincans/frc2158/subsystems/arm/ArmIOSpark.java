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

package org.austincans.frc2158.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ArmIOSpark implements ArmIO {
  /* Motor */
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private SparkFlexConfig config;

  private int canID = 0;
  private int statorCurrentLimit = 0;
  private boolean brakeMode = false;

  public ArmIOSpark() {
    // Initialize motor controller
    motor = new SparkFlex(canID, MotorType.kBrushless);
    
    config = new SparkFlexConfig();
    config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Set current limits
    config.smartCurrentLimit((int) statorCurrentLimit);

    // Save configuration
    motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.currentVoltage = motor.getBusVoltage();
    inputs.position = encoder.getPosition();
    inputs.temperature = motor.getMotorTemperature();
    inputs.velocity = encoder.getVelocity();
    inputs.currentAmps = motor.getOutputCurrent();
  }
}
