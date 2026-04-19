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

package org.austincans.frc2026.subsystems.intake.roller;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import org.austincans.frc2026.subsystems.intake.IntakeConstants;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;

public class RollerIOSparkFlex implements RollerIO {
  private final SparkFlex motor;
  private final RelativeEncoder encoder;

  public RollerIOSparkFlex() {
    motor = new SparkFlex(IntakeConstants.rollerMotorId, MotorType.kBrushless);
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.motorVoltage = motor.getBusVoltage();
    inputs.temperatureCelsius = motor.getMotorTemperature();
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  }

  @Override
  public void applyOutputs(RollerIOOutputs outputs) {
    motor.setVoltage(outputs.voltage);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Roller Motor", motor);
  }
}
