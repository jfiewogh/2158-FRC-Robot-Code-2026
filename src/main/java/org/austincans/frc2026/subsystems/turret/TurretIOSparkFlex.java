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

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public class TurretIOSparkFlex extends TurretIOReal {
  private final SparkFlex motor;
  private final RelativeEncoder relativeEncoder;
  private final SparkClosedLoopController controller;

  public TurretIOSparkFlex() {
    super();
    motor =
        new SparkFlex(
            TurretConstants.motorId,
            TurretConstants.isBrushlessMotor ? MotorType.kBrushless : MotorType.kBrushed);

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.softLimit.forwardSoftLimitEnabled(TurretConstants.forwardSoftLimitEnabled);
    motorConfig.softLimit.forwardSoftLimit(3);
    motorConfig.softLimit.reverseSoftLimitEnabled(TurretConstants.reverseSoftLimitEnabled);
    motorConfig.softLimit.reverseSoftLimit(-3);
    motorConfig.closedLoop.maxMotion.cruiseVelocity(TurretConstants.maxVelocityRotationsPerSec);
    motorConfig.closedLoop.maxMotion.maxAcceleration(
        TurretConstants.maxAccelerationRotationsPerSecSq);
    motorConfig.smartCurrentLimit(TurretConstants.smartCurrentLimit);
    motorConfig.idleMode(TurretConstants.isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(
        motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    relativeEncoder = motor.getEncoder();
    relativeEncoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());

    controller = motor.getClosedLoopController();
    controller.setSetpoint(0, ControlType.kPosition);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    super.updateInputs(inputs);
    inputs.positionRad = Units.rotationsToRadians(relativeEncoder.getPosition() / TurretConstants.gearing);
    inputs.motorPositionRotations = relativeEncoder.getPosition();
    inputs.motorVelocityRotationsPerSec = relativeEncoder.getVelocity() / 60;
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.voltage = motor.getBusVoltage();
    inputs.temperatureCelsius = motor.getMotorTemperature();
    relativeEncoder.setPosition(encoder.getPosition().getValueAsDouble());
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

  private void setVoltage(double voltage) {
    controller.setSetpoint(voltage, ControlType.kVoltage);
  }

  private void setVelocity(double velocityRadPerSec) {
    controller.setSetpoint(
        Units.radiansToRotations(velocityRadPerSec) * 60, 
        ControlType.kMAXMotionVelocityControl);
  }

  private void setDesiredAngle(Rotation2d angle) {
    double rotations =
        MathUtil.inputModulus(
            angle.getRotations(),
            TurretConstants.turretMinimumRotation,
            TurretConstants.turretMaxRotation);
    controller.setSetpoint(rotations * TurretConstants.gearing, ControlType.kPosition);
  }

  @Override
  public void resetAngle(Rotation2d newAngle) {
    relativeEncoder.setPosition(newAngle.getRotations() * TurretConstants.gearing);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Turret", motor);
  }
}
