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
import java.util.ArrayList;
import java.util.List;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOSparkFlex;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingSparkFlex;

public class ElevatorIOSpark implements ElevatorIO {
  private final boolean brakeMode = true;

  private final MotorIOSparkFlex motorIO;
  private final SelfCheckingSparkFlex motorSelfCheck;
  private final RelativeEncoder encoder;
  private SparkFlexConfig config;

  public ElevatorIOSpark(ElevatorConfig elevatorConfig) {
    motorIO = MotorFactory.createSparkFlex(elevatorConfig.motorConfig);
    motorSelfCheck = new SelfCheckingSparkFlex("Elevator Motor", motorIO.getMotor());
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
  public void runVoltage(double voltage) {
    motorIO.setVoltage(voltage);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.heightMeters = encoder.getPosition();
    inputs.velocityMetersPerSec = encoder.getVelocity();
    motorIO.updateInputs(inputs.motorInputs);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {

  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();

    List<HardwareFault> motorFaults = motorSelfCheck.checkForFaults();
    if (motorFaults != null) {
      faults.addAll(motorFaults);
    }

    if (faults.isEmpty()) {
      return List.of();
    } else {
      return faults;
    }
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(motorSelfCheck);
  }
}
