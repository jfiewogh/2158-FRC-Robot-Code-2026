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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2026.subsystems.selfcheck.SelfCheckingSparkFlex;

public class TurretIOSparkFlex extends TurretIOReal {
  private final SparkFlex motor;
  private final RelativeEncoder relativeEncoder;
  private final SparkClosedLoopController controller;
  private final SelfCheckingSparkFlex selfCheckingMotor;

  public TurretIOSparkFlex() {
    super();
    motor =
        new SparkFlex(
            TurretConstants.turretMotorId,
            TurretConstants.isBrushlessMotor ? MotorType.kBrushless : MotorType.kBrushed);
    selfCheckingMotor = new SelfCheckingSparkFlex("Turret", motor);

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.closedLoop.pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
    motorConfig.closedLoop.feedForward.kS(TurretConstants.kS);
    motorConfig.closedLoop.feedForward.kV(TurretConstants.kV);
    motorConfig.closedLoop.feedForward.kA(TurretConstants.kA);
    motorConfig.softLimit.forwardSoftLimitEnabled(TurretConstants.forwardSoftLimitEnabled);
    motorConfig.softLimit.forwardSoftLimit(TurretConstants.forwardSoftLimit / TurretConstants.gearing);
    motorConfig.softLimit.reverseSoftLimitEnabled(TurretConstants.reverseSoftLimitEnabled);
    motorConfig.softLimit.reverseSoftLimit(TurretConstants.reverseSoftLimit / TurretConstants.gearing);
    motorConfig.closedLoop.maxMotion.cruiseVelocity(TurretConstants.maxVelocityRotationsPerSec);
    motorConfig.closedLoop.maxMotion.maxAcceleration(TurretConstants.maxAccelerationRotationsPerSecSq);
    motorConfig.smartCurrentLimit(TurretConstants.smartCurrentLimit);
    motorConfig.idleMode(TurretConstants.isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    motorConfig.encoder.positionConversionFactor(1 / TurretConstants.gearing);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    relativeEncoder = motor.getEncoder();
    relativeEncoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());

    controller = motor.getClosedLoopController();
  }

  @Override
  public void setVoltage(double voltage) {
    controller.setSetpoint(voltage, ControlType.kVoltage);
  }

  @Override
  public void setDesiredAngle(Rotation2d angle) {
    double rotations = MathUtil.inputModulus(angle.getRotations(), TurretConstants.turretMaxRotation, TurretConstants.turretMinimumRotation);
    controller.setSetpoint(rotations, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void setVelocity(double radiansPerSec) {
    controller.setSetpoint(Units.radiansToRotations(radiansPerSec), ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    super.updateInputs(inputs);
    inputs.motorPositionRotations = relativeEncoder.getPosition();
    inputs.motorVelocityRotationsPerSec = relativeEncoder.getVelocity() / 60;
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.voltage = motor.getBusVoltage();
    inputs.temperatureCelsius = motor.getMotorTemperature();
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();
    List<HardwareFault> deployFaults = selfCheckingMotor.checkForFaults();
    if (deployFaults != null) {
      faults.addAll(deployFaults);
    }
    return faults.isEmpty() ? null : faults;
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(selfCheckingMotor);
  }
}
