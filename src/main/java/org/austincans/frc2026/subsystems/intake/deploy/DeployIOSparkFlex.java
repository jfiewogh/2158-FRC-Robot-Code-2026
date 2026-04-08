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

package org.austincans.frc2026.subsystems.intake.deploy;

import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.subsystems.intake.IntakeConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;

public class DeployIOSparkFlex implements DeployIO {
  private final SparkFlex leftDeployMotor;
  private final SparkFlex rightDeployMotor;
  private final RelativeEncoder rightDeployEncoder;

  public DeployIOSparkFlex() {
    leftDeployMotor = new SparkFlex(IntakeConstants.leftDeployMotorId, MotorType.kBrushless);
    rightDeployMotor = new SparkFlex(IntakeConstants.rightDeployMotorId, MotorType.kBrushless);
    rightDeployEncoder = rightDeployMotor.getEncoder();
  }

  @Override
  public void updateInputs(DeployIOInputs inputs) {
    inputs.motorVoltage = rightDeployMotor.getBusVoltage();
    inputs.positionRad = Units.rotationsToRadians(rightDeployEncoder.getPosition());
    inputs.temperatureCelsius = rightDeployMotor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(DeployIOOutputs outputs) {
    leftDeployMotor.setVoltage(outputs.voltage);
    rightDeployMotor.setVoltage(outputs.voltage);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Left Deploy Motor", leftDeployMotor);
    subsystem.registerHardware("Right Deploy Motor", leftDeployMotor);
  }
}
