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

package org.austincans.frc2026.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2026.subsystems.selfcheck.SelfCheckingSparkFlex;

/** Add your docs here. */
public class IntakeIOSparkFlex implements IntakeIO {
  private final SparkFlex intakeMotor;
  private final SelfCheckingSparkFlex intakeMotorSelfCheck;

  public IntakeIOSparkFlex() {
    // Initialize motor controllers
    intakeMotor = new SparkFlex(28, MotorType.kBrushless);
    // FIX ME: SparkFlex keeps initializing as default int

    intakeMotorSelfCheck = new SelfCheckingSparkFlex("Intake Motor", intakeMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorVoltage = intakeMotor.getBusVoltage();
    inputs.intakeTemperatureC = intakeMotor.getMotorTemperature();
  }

  @Override
  public void setRelativeVelocity(double velocity) {
    intakeMotor.set(velocity);
  }

  @Override
  public void setVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(intakeMotorSelfCheck);
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();
    List<HardwareFault> intakeFault = intakeMotorSelfCheck.checkForFaults();
    if (intakeFault != null) {
      faults.addAll(intakeFault);
    }

    if (faults.isEmpty()) {
      return List.of();
    } else {
      return faults;
    }
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    switch (outputs.mode) {
      case VELOCITY:
        setRelativeVelocity(outputs.velocity);
        break;
      case VOLTAGE:
        setVoltage(outputs.voltage);
        break;
    }
  }
}
