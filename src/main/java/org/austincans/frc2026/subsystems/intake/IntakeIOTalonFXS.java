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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.List;

import org.austincans.lib.util.ConfigUtil;
import org.austincans.lib.math.Angle;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingSparkFlex;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingTalonFXS;

/** Add your docs here. */
public class IntakeIOTalonFXS implements IntakeIO {
  private final TalonFXS deployMotor1;
  private final TalonFXS deployMotor2;
  private final SparkFlex intakeMotor;

  private final SelfCheckingTalonFXS deploy1MotorSelfCheck;
  private final SelfCheckingTalonFXS deploy2MotorSelfCheck;
  private final SelfCheckingSparkFlex intakeMotorSelfCheck;

  private final TalonFXSConfiguration config;
  private final CANcoder deployEncoder;
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
// TODO: SEPERATE DEPLOY MOTORS AND ROLLER MOTORS
  public IntakeIOTalonFXS() {
    // Initialize motor controller
    deployMotor1 = new TalonFXS(IntakeConstants.deployCanID1, IntakeConstants.intakeCANBus);
    deployMotor2 = new TalonFXS(IntakeConstants.deployCanID2, IntakeConstants.intakeCANBus);
    intakeMotor =
        new SparkFlex(ConfigUtil.getConfigInt("INTAKE_DEPLOY_ENCODER_ID", 0), MotorType.kBrushless);

    deploy1MotorSelfCheck = new SelfCheckingTalonFXS("Intake Deploy Motor 1", deployMotor1);
    deploy2MotorSelfCheck = new SelfCheckingTalonFXS("Intake Deploy Motor 2", deployMotor2);
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

    // Configure encoder
    deployEncoder = new CANcoder(ConfigUtil.getConfigInt("ARM_ENCODER_ID", 0));

    config.ExternalFeedback.FeedbackRemoteSensorID = deployEncoder.getDeviceID();
    config.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.FusedCANcoder;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    deployMotor1.getConfigurator().apply(config);
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    deployMotor2.getConfigurator().apply(config);
    deployMotor2.setControl(new Follower(IntakeConstants.deployCanID1, MotorAlignmentValue.Aligned));
    deployEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.deployMotorVoltage = deployMotor1.getMotorVoltage().getValueAsDouble();
    inputs.deployPositionRad = deployMotor1.getPosition().getValue().in(Units.Radians);
    inputs.deployTemperatureC = deployMotor1.getDeviceTemp().getValueAsDouble();
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

  public void setDeployAngle(Angle angle) {
    deployMotor1.setControl(positionRequest.withPosition(angle.toRotations()));
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(deploy1MotorSelfCheck, deploy2MotorSelfCheck, intakeMotorSelfCheck);
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();

    List<HardwareFault> deployFault = deploy2MotorSelfCheck.checkForFaults();
    deployFault.addAll(deploy1MotorSelfCheck.checkForFaults());
    if (deployFault != null) {
      faults.addAll(deployFault);
    }

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

  @Override
  public void deploy() {
    deployMotor1.setVoltage(5); // TODO: CHANGE ME
  }

  @Override
  public void retract() {
    deployMotor2.setVoltage(-5);
  }
}
