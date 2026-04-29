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

package org.austincans.lib.subsystems.turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOSparkFlex;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public class TurretIOSparkFlex extends TurretIOReal {
  private final MotorIOSparkFlex motorIO;
  private final RelativeEncoder relativeEncoder;

  public TurretIOSparkFlex(TurretConfig turretConfig) {
    super(turretConfig.encoderConfig);

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.softLimit.forwardSoftLimitEnabled(TurretConstants.forwardSoftLimitEnabled);
    motorConfig.softLimit.forwardSoftLimit(3);
    motorConfig.softLimit.reverseSoftLimitEnabled(TurretConstants.reverseSoftLimitEnabled);
    motorConfig.softLimit.reverseSoftLimit(-3);
    motorConfig.closedLoop.maxMotion.cruiseVelocity(TurretConstants.maxVelocityRotationsPerSec);
    motorConfig.closedLoop.maxMotion.maxAcceleration(
        TurretConstants.maxAccelerationRotationsPerSecSq);
    motorConfig.smartCurrentLimit(TurretConstants.smartCurrentLimit);
    motorConfig.idleMode(TurretConstants.isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    motorIO = MotorFactory.createSparkFlex(turretConfig.motorSparkConfig,motorConfig);

    relativeEncoder = motorIO.getMotor().getEncoder();
    relativeEncoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    super.updateInputs(inputs);
    motorIO.updateInputs(inputs.motorInputs);
    relativeEncoder.setPosition(encoder.getPosition().getValueAsDouble());
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    motorIO.applyOutputs(outputs.motorOutputs);
  }

  @Override
  public void resetAngle(Rotation2d newAngle) {
    relativeEncoder.setPosition(newAngle.getRotations() * TurretConstants.gearing);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Turret", motorIO.getMotor());
  }
}