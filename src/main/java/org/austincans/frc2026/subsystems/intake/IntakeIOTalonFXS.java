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

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2026.subsystems.selfcheck.SelfCheckingSparkFlex;
import org.austincans.frc2026.util.ConfigUtil;

/** Add your docs here. */
public class IntakeIOTalonFXS implements IntakeIO {
  private final SelfCheckingSparkFlex intakeMotorSelfCheck;
  private final SparkFlex intakeMotor;

  private final TalonFXSConfiguration config;
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  // TODO: SEPERATE DEPLOY MOTORS AND ROLLER MOTORS
  public IntakeIOTalonFXS() {
    // Initialize motor controller
    intakeMotor =
        new SparkFlex(ConfigUtil.getConfigInt("INTAKE_DEPLOY_ENCODER_ID", 0), MotorType.kBrushless);
    intakeMotorSelfCheck = new SelfCheckingSparkFlex("Intake Motor", intakeMotor);

    config = new TalonFXSConfiguration();
    config.MotorOutput.NeutralMode =
        IntakeConstants.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.Slot0.kP = IntakeConstants.kP;
    config.Slot0.kI = IntakeConstants.kI;
    config.Slot0.kD = IntakeConstants.kD;
    config.Slot0.kS = IntakeConstants.kS;
    config.Slot0.kG = IntakeConstants.kG;
    config.Slot0.kV = IntakeConstants.kV;
    config.Slot0.kA = IntakeConstants.kA;

    // // Configure encoder
    // deployEncoder = new CANcoder(ConfigUtil.getConfigInt("ARM_ENCODER_ID", 0));
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
      return null;
    } else {
      return faults;
    }
  }
}
