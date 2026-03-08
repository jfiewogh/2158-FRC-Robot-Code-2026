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

package org.austincans.frc2026.subsystems.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.ArrayList;
import java.util.List;

import org.austincans.frc2026.subsystems.intake.IntakeConstants;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2026.subsystems.selfcheck.SelfCheckingSparkFlex;

public class FlywheelIOSparkFlex extends FlywheelIOReal {
  private final SparkFlex motor1 = new SparkFlex(FlywheelConstants.motor1Id, MotorType.kBrushless);
  private final SelfCheckingSparkFlex selfCheckMotor1 =
      new SelfCheckingSparkFlex("flywheel motor 1", motor1);
  private final SparkFlex motor2 = new SparkFlex(FlywheelConstants.motor2Id, MotorType.kBrushless);
  private final SelfCheckingSparkFlex selfCheckMotor2 =
      new SelfCheckingSparkFlex("flywheel motor 2", motor2);

  // private final CANcoder encoder =
  //     new CANcoder(FlywheelConstants.encoderId, FlywheelConstants.canBus);
  private final SparkClosedLoopController controller = motor1.getClosedLoopController();
  private final SparkFlexConfig config = new SparkFlexConfig();

  public FlywheelIOSparkFlex() {
    config.follow(motor1, true);
    // config.inverted(true);
    motor2.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.appliedVolts = motor1.getBusVoltage();
    inputs.velocityRPM = motor1.getEncoder().getVelocity() * FlywheelConstants.flywheelGearRation;
    // inputs.velocityRPM = encoder.getVelocity().getValue().in(Units.RPM);
    inputs.temperatureC = motor1.getMotorTemperature();
  }

  @Override
  public void runVoltage(double volts) {
    motor1.setVoltage(volts);
  }

  @Override
  public void runRelativeVelocity(double velocityRPM) {
    motor1.set(velocityRPM);
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();
    List<HardwareFault> motorFaults = selfCheckMotor1.checkForFaults();
    motorFaults.addAll(selfCheckMotor2.checkForFaults());
    if (motorFaults != null) {
      faults.addAll(motorFaults);
    }
    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(selfCheckMotor1, selfCheckMotor2);
  }
}
