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

package org.austincans.lib.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOTalonFX;
import org.austincans.lib.motors.MotorIO.MotorIOOutputMode;
import org.austincans.lib.motors.MotorIO.MotorIOOutputs;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public class TurretIOTalonFX extends TurretIOReal {
  private final MotorIOTalonFX motorIO;

  public TurretIOTalonFX(TurretConfig turretConfig) {
    super(turretConfig.encoderConfig);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0.kP = TurretConstants.kP.getAsDouble();
    motorConfig.Slot0.kI = TurretConstants.kI.getAsDouble();
    motorConfig.Slot0.kD = TurretConstants.kD.getAsDouble();
    motorConfig.Slot0.kV = TurretConstants.kV.getAsDouble();
    motorConfig.Slot0.kS = TurretConstants.kS.getAsDouble();
    motorConfig.Slot0.kA = TurretConstants.kA.getAsDouble();
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
    motorIO = MotorFactory.createTalonFX(turretConfig.motorTalonConfig, motorConfig);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    super.updateInputs(inputs);
    motorIO.updateInputs(inputs.motorInputs);
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs){
    motorIO.applyOutputs(outputs.motorOutputs);
  }

  private void setDesiredAngle(Rotation2d angle, double accelerationRadPerSecSq) {
    double positionRad = angle.getRadians();
    double positionRotations = Units.radiansToRotations(positionRad);
    positionRotations =
        MathUtil.inputModulus(
            angle.getRotations(),
            TurretConstants.turretMinimumRotation,
            TurretConstants.turretMaxRotation);
    MotorIOOutputs outputs = new MotorIOOutputs();
    outputs.setPosition(Rotations.of(positionRotations));
    outputs.mode = MotorIOOutputMode.POSITION;
    motorIO.applyOutputs(outputs);
  }
  /**
   * Set pivot angular velocity with acceleration.
   * @param velocityDegPerSec The target velocity in radians per second
   * @param acceleration The acceleration in radians per second squared
   */
  private void setVelocity(double velocityRadPerSec, double acceleration) {
    double velocityRotationsPerSec = Units.rotationsToRadians(velocityRadPerSec);
    MotorIOOutputs outputs = new MotorIOOutputs();
    outputs.setVelocity(RadiansPerSecond.of(velocityRotationsPerSec));
    outputs.mode = MotorIOOutputMode.VELOCITY;
    motorIO.applyOutputs(outputs);
  }

  @Override
  public void resetAngle(Rotation2d angle) {
    MotorIOOutputs outputs = new MotorIOOutputs();
    outputs.setPosition(Rotations.of(angle.getRotations() * TurretConstants.gearing));
    outputs.mode = MotorIOOutputMode.POSITION;
    motorIO.applyOutputs(outputs);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Turret", motorIO.getMotor());
  }
}