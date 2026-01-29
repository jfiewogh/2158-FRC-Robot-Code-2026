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
// Updated on Thursday January 29, 2026 at 17:14:42 CST

package org.austincans.frc2158.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingTalonFXS;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;
import org.austincans.frc2158.util.ConfigUtil;

public class ArmIOTalonFXS implements ArmIO {
  /* Motor */
  private final TalonFXS motor;
  private SelfCheckingTalonFXS motorSelfCheck;
  private final TalonFXSConfiguration config;
  private final CANcoder encoder;

  private int canID = ConfigUtil.getConfigInt("armID", 0);
  private int statorCurrentLimit = ConfigUtil.getConfigInt("armCurrentLimit", 0);
  private boolean brakeMode = ConfigUtil.getConfigBoolean("armIsBrakeMode", false);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  public ArmIOTalonFXS() {
    // Initialize motor controller
    motor = new TalonFXS(canID, ArmConstants.armCANBus);
    motorSelfCheck = new SelfCheckingTalonFXS("Arm Motor", motor);

    config = new TalonFXSConfiguration();
    config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;

    // Configure encoder
    encoder = new CANcoder(ConfigUtil.getConfigInt("armEncoderID", 0));

    config.ExternalFeedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.FusedCANcoder;

    motor.getConfigurator().apply(config);

    encoder.setPosition(0);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setPosition(double position) {
    motor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void setVelocity(double velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  public void stopMotor() {
    setVoltage(0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.currentVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.position = motor.getPosition().getValueAsDouble();
    inputs.temperature = motor.getDeviceTemp().getValueAsDouble();
    inputs.velocity = motor.getVelocity().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = motorSelfCheck.checkForFaults();
    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }
}
