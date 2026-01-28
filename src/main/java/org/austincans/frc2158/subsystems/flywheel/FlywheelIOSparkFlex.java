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
// Updated on Tuesday January 27, 2026 at 19:20:07 CST

package org.austincans.frc2158.subsystems.flywheel;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingSparkFlex;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;

public class FlywheelIOSparkFlex implements FlywheelIO {
  private final SparkFlex motor = new SparkFlex(FlywheelConstants.motorId, MotorType.kBrushless);
  private final SelfCheckingSparkFlex selfCheckMotor = new SelfCheckingSparkFlex("flywheel", motor);

  private final CANcoder encoder =
      new CANcoder(FlywheelConstants.encoderId, FlywheelConstants.canBus);
  private final SparkClosedLoopController controller = motor.getClosedLoopController();

  public FlywheelIOSparkFlex() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.appliedVolts = motor.getBusVoltage();
    inputs.velocityRPM = encoder.getVelocity().getValue().in(Units.RPM);
    inputs.temperatureC = motor.getMotorTemperature();
  }

  @Override
  public void runVoltage(double volts) {
    controller.setSetpoint(volts, ControlType.kVoltage);
  }

  @Override
  public void runVelocity(double velocityRPM) {
    controller.setSetpoint(velocityRPM, ControlType.kVelocity);
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();
    List<SubsystemFault> motorFaults = selfCheckMotor.checkForFaults();
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
