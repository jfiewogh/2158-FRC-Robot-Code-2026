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
// Updated on Tuesday January 13, 2026 at 12:34:16 CST

package org.austincans.frc2158.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingTalonFXS;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;
import org.austincans.frc2158.util.ConfigUtil;

public class TurretIOTalonFX implements TurretIO {
  private TalonFX turretMotor;
  private CANcoder turretEncoder;
  private SelfCheckingTalonFXS selfCheckingTalonFXS;

  private CANcoderConfiguration turretEncoderConfig;
  // Feedforward
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          kS, // kS
          0, // kG - Pivot doesn't need gravity compensation
          kV, // kV
          kA // kA
          );

  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private TurretIOInputs inputs;

  public TurretIOTalonFX() {
    turretMotor = new TalonFX(TurretConstants.turretMotorId, TurretConstants.turretCANBus);
    turretEncoder = new CANcoder(TurretConstants.turretEncoderId, TurretConstants.turretCANBus);
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    turretEncoderConfig = new CANcoderConfiguration();

    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    motorConfig.Slot0.kV = kV;
    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kA = kA;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ConfigUtil.getConfigDouble("turretSoftLimit", 0);
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ConfigUtil.getConfigDouble("turretHardLimit", 0);

    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();

    turretEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    turretMotor.getConfigurator().apply(motorConfig);
    turretEncoder.getConfigurator().apply(turretEncoderConfig);
    turretMotor.setPosition(0);
    turretEncoder.setPosition(0);
  }

  public void setDesiredAngle(double angleDegrees) {
    setDesiredAngle(angleDegrees, 0);
  }

  public void setDesiredAngle(double angleDegrees, double acceleration) {
    // Convert degrees to rotations
    double angleRadians = Units.degreesToRadians(angleDegrees);
    double positionRotations = angleRadians / (2.0 * Math.PI);

    double ffVolts = feedforward.calculate(inputs.velocityDegPerSec, acceleration);
    // turretMotor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
    turretMotor.setControl(positionRequest.withPosition(positionRotations));
  }

  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, 0);
  }

  /**
   * Set pivot angular velocity with acceleration.
   *
   * @param velocityDegPerSec The target velocity in degrees per second
   * @param acceleration The acceleration in degrees per second squared
   */
  public void setVelocity(double velocityDegPerSec, double acceleration) {
    // Convert degrees/sec to rotations/sec
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    double ffVolts = feedforward.calculate(inputs.velocityDegPerSec, acceleration);
    // turretMotor.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
    turretMotor.setControl(velocityRequest.withVelocity(velocityRotations));
  }

  public void setVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  public void updateInputs(TurretIOInputs inputs) {
    inputs.currentAmps = turretMotor.getStatorCurrent().getValueAsDouble();
    inputs.voltage = turretMotor.getMotorVoltage().getValueAsDouble();
    inputs.positionDeg = turretEncoder.getPosition().getValueAsDouble();
    inputs.temperatureC = turretMotor.getDeviceTemp().getValueAsDouble();
    inputs.velocityDegPerSec = turretEncoder.getVelocity().getValueAsDouble();
  }

  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    List<SubsystemFault> deployFault = selfCheckingTalonFXS.checkForFaults();
    if (deployFault != null) {
      faults.addAll(deployFault);
    }

    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }
}
