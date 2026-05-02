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

package org.austincans.lib.subsystems.elevator;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOSparkFlex;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public class ElevatorIOSpark implements ElevatorIO {
  private final boolean brakeMode = true;

  private final MotorIOSparkFlex motorIO;
  private final RelativeEncoder encoder;
  private SparkFlexConfig config;

  public ElevatorIOSpark(ElevatorConfig elevatorConfig) {
    motorIO = MotorFactory.createSparkFlex(elevatorConfig.motorConfigSpark,elevatorConfig.sparkFlexConfig);
    config = new SparkFlexConfig();
    config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure encoder
    encoder = motorIO.getMotor().getEncoder();
    config.encoder.positionConversionFactor(
        Math.PI * 2 * ElevatorConfig.drumRadius / ElevatorConfig.gearRatio);
    config.encoder.velocityConversionFactor(
        Math.PI * 2 * ElevatorConfig.drumRadius / (ElevatorConfig.gearRatio * 60.0));

    encoder.setPosition(0);

    // Set current limits
    config.smartCurrentLimit((int) ElevatorConfig.statorCurrentLimit);

    // Save configuration
    motorIO.getMotor().configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.heightMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity();
    motorIO.updateInputs(inputs.motorInputs);
  }

  @Override
  public void applyOutputs(ElevatorIOOutputs outputs) {
    motorIO.applyOutputs(outputs.motorOutputs);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Flywheel", motorIO.getMotor());
  }

}
