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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;

public class TurretIOTalonFX extends TurretIOReal {
  private final TalonFX motor;
  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;

  public TurretIOTalonFX() {
    super();

    motor = new TalonFX(TurretConstants.motorId, TurretConstants.canBus);
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

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
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    super.updateInputs(inputs);
    inputs.motorPositionRotations = motor.getPosition().getValueAsDouble();
    inputs.positionRad = Units.rotationsToRadians(inputs.motorPositionRotations / TurretConstants.gearing);
    inputs.motorVelocityRotationsPerSec = motor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs){
    switch (outputs.mode){
      case VELOCITY -> {
        setVelocity(outputs.velocityRadPerSec);
      }
      case VOLTAGE -> {
        setVoltage(outputs.voltage);
      }
      case ANGLE -> {
        setDesiredAngle(outputs.angle);
      }
    }
  }

  private void setDesiredAngle(Rotation2d angle) {
    setDesiredAngle(angle, 0);
  }

  private void setDesiredAngle(Rotation2d angle, double accelerationRadPerSecSq) {
    double positionRad = angle.getRadians();
    double positionRotations = Units.radiansToRotations(positionRad);
    positionRotations =
        MathUtil.inputModulus(
            angle.getRotations(),
            TurretConstants.turretMinimumRotation,
            TurretConstants.turretMaxRotation);
    motor.setControl(positionRequest.withPosition(positionRotations * TurretConstants.gearing));
  }

  private void setVelocity(double velocityRadPerSec) {
    setVelocity(velocityRadPerSec, 0);
  }

  /**
   * Set pivot angular velocity with acceleration.
   * @param velocityDegPerSec The target velocity in radians per second
   * @param acceleration The acceleration in radians per second squared
   */
  private void setVelocity(double velocityRadPerSec, double acceleration) {
    double velocityRotationsPerSec = Units.rotationsToRadians(velocityRadPerSec);
    motor.setControl(velocityRequest.withVelocity(velocityRotationsPerSec));
  }

  private void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void resetAngle(Rotation2d angle) {
    motor.setPosition(angle.getRotations() * TurretConstants.gearing);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Turret", motor);
  }
}
