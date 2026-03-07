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

package org.austincans.frc2026.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingTalonFXS;

public class TurretIOTalonFX extends TurretIOReal {
  private static final boolean feedforwardEnabled = true;

  private TalonFX turretMotor;
  private SelfCheckingTalonFXS selfCheckingTalonFXS;

  // Feedforward
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          TurretConstants.kS, // kS
          0, // kG - Pivot doesn't need gravity compensation
          TurretConstants.kV, // kV
          TurretConstants.kA // kA
          );

  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private TurretIOInputs inputs;

  public TurretIOTalonFX() {
    super();

    turretMotor = new TalonFX(TurretConstants.turretMotorId, TurretConstants.turretCANBus);
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0.kP = TurretConstants.kP;
    motorConfig.Slot0.kI = TurretConstants.kI;
    motorConfig.Slot0.kD = TurretConstants.kD;
    motorConfig.Slot0.kV = TurretConstants.kV;
    motorConfig.Slot0.kS = TurretConstants.kS;
    motorConfig.Slot0.kA = TurretConstants.kA;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        TurretConstants.forwardSoftLimitEnabled;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.forwardSoftLimit;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable =
        TurretConstants.reverseSoftLimitEnabled;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.reverseSoftLimit;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode =
        TurretConstants.isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
  }

  @Override
  public void setDesiredAngle(Rotation2d angleDegrees) {
    setDesiredAngle(angleDegrees, 0);
  }

  public void setDesiredAngle(Rotation2d angle, double accelerationRadPerSecSq) {
    double positionRad = angle.getRadians();
    double positionRotations = Units.radiansToRotations(positionRad);
    positionRotations = MathUtil.inputModulus(angle.getRotations(), TurretConstants.turretMinimumRotation, TurretConstants.turretMaxRotation);
    // positionRotations = MathUtil.clamp(positionRotations, TurretConstants.turretMinimumRotation, TurretConstants.turretMaxRotation);
    if (feedforwardEnabled) {
      // TODO: feedforward seems to take in parameters of positionRad and velocityRadPerSec
      double ffVolts = feedforward.calculate(inputs.velocityRadPerSec, accelerationRadPerSecSq);
      turretMotor.setControl(
          positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
    } else {
      turretMotor.setControl(positionRequest.withPosition(positionRotations));
    }
  }

  @Override
  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, 0);
  }

  /**
   * Set pivot angular velocity with acceleration.
   *
   * @param velocityDegPerSec The target velocity in radians per second
   * @param acceleration The acceleration in radians per second squared
   */
  public void setVelocity(double velocityRadPerSec, double acceleration) {
    double velocityRotationsPerSec = Units.rotationsToRadians(velocityRadPerSec);

    if (feedforwardEnabled) {
      // TODO: feedforward seems to take in parameters of positionRad and velocityRadPerSec
      // Also, should we use the velocityRadPerSec from the parameter or inputs?
      double ffVolts = feedforward.calculate(inputs.velocityRadPerSec, acceleration);
      turretMotor.setControl(
          velocityRequest.withVelocity(velocityRotationsPerSec).withFeedForward(ffVolts));
    } else {
      turretMotor.setControl(velocityRequest.withVelocity(velocityRotationsPerSec));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    super.updateInputs(inputs);
    inputs.currentAmps = turretMotor.getStatorCurrent().getValueAsDouble();
    inputs.voltage = turretMotor.getMotorVoltage().getValueAsDouble();
    inputs.temperatureCelsius = turretMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();

    List<HardwareFault> deployFault = selfCheckingTalonFXS.checkForFaults();
    if (deployFault != null) {
      faults.addAll(deployFault);
    }

    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(selfCheckingTalonFXS);
  }
}
