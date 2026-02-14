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
// Updated on Saturday February 14, 2026 at 11:44:36 CST

package org.austincans.frc2158.subsystems.elevator;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingSparkFlex;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;

public class ElevatorIOSpark implements ElevatorIO {
  // TODO: move constants to config

  private final int canID = 0;
  private final boolean brakeMode = true;

  private final SparkFlex motor;
  private final SelfCheckingSparkFlex motorSelfCheck;
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
    motorSelfCheck = new SelfCheckingSparkFlex("Elevator Motor", motor);
    config = new SparkFlexConfig();
    config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure encoder
    encoder = motor.getEncoder();
    config.encoder.positionConversionFactor(
        Math.PI * 2 * ElevatorConstants.drumRadius / ElevatorConstants.gearRatio);
    config.encoder.velocityConversionFactor(
        Math.PI * 2 * ElevatorConstants.drumRadius / (ElevatorConstants.gearRatio * 60.0));

    encoder.setPosition(0);

    // Set current limits
    config.smartCurrentLimit((int) statorCurrentLimit);

    // Save configuration
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void runVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.heightMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity();

    inputs.voltage = motor.getBusVoltage();
    inputs.current = motor.getOutputCurrent();
    inputs.temperatureC = motor.getMotorTemperature();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    List<SubsystemFault> motorFaults = motorSelfCheck.checkForFaults();
    if (motorFaults != null) {
      faults.addAll(motorFaults);
    }

    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }
}
