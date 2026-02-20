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
// Updated on Thursday February 05, 2026 at 19:51:21 CST

package org.austincans.frc2158.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2158.math.Angle;
import org.austincans.frc2158.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingTalonFXS;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;

/** Add your docs here. */
public class IntakeIOTalonFXS implements IntakeIO {
  private final TalonFXS deployMotor;
  private final TalonFXS intakeMotor;

  private final SelfCheckingTalonFXS deployMotorSelfCheck;
  private final SelfCheckingTalonFXS intakeMotorSelfCheck;

  private final TalonFXSConfiguration config;
  private final CANcoder deployEncoder;
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public IntakeIOTalonFXS() {
    // Initialize motor controller
    deployMotor = new TalonFXS(IntakeConstants.deployMotorID, IntakeConstants.intakeCANBus);
    intakeMotor = new TalonFXS(IntakeConstants.intakeMotorID, IntakeConstants.intakeCANBus);

    deployMotorSelfCheck = new SelfCheckingTalonFXS("Intake Deploy Motor", deployMotor);
    intakeMotorSelfCheck = new SelfCheckingTalonFXS("Intake Motor", intakeMotor);

    config = new TalonFXSConfiguration();
    config.MotorOutput.NeutralMode =
        deployBreakEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;

    // Configure encoder
    deployEncoder = new CANcoder(IntakeConstants.deployEncoderID);

    config.ExternalFeedback.FeedbackRemoteSensorID = deployEncoder.getDeviceID();
    config.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.FusedCANcoder;

    deployMotor.getConfigurator().apply(config);

    deployEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.deployMotorVoltage = deployMotor.getMotorVoltage().getValueAsDouble();
    inputs.deployPositionRad = deployMotor.getPosition().getValue().in(Units.Radians);
    inputs.deployTemperatureC = deployMotor.getDeviceTemp().getValueAsDouble();
    inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeTemperatureC = intakeMotor.getDeviceTemp().getValueAsDouble();
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
  public void setDeployAngle(Angle angle) {
    deployMotor.setControl(positionRequest.withPosition(angle.toRotations()));
  }

  @Override
  public List<SelfChecking> getHardware(){
    return List.of(deployMotorSelfCheck, intakeMotorSelfCheck);
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    List<SubsystemFault> deployFault = deployMotorSelfCheck.checkForFaults();
    if (deployFault != null) {
      faults.addAll(deployFault);
    }

    List<SubsystemFault> intakeFault = intakeMotorSelfCheck.checkForFaults();
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
