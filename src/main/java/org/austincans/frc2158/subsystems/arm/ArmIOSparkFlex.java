// Copyright (c) 2025 the ausTIN CANs
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
// Updated on Wednesday November 19, 2025 at 19:28:08 CST

package org.austincans.frc2158.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import org.austincans.frc2158.util.ConfigUtil;

public class ArmIOSparkFlex implements ArmIO {
  /* Motor */
  private final SparkFlex motor;
  private final RelativeEncoder encoder;
  private SparkFlexConfig config;
  private final SparkClosedLoopController controller;

  private int canID = ConfigUtil.getConfigInt("armID", 0);
  private int statorCurrentLimit = ConfigUtil.getConfigInt("armCurrentLimit", 0);
  private boolean brakeMode = ConfigUtil.getConfigBoolean("armIsBrakeMode", false);

  public ArmIOSparkFlex() {
    // Initialize motor controller
    motor = new SparkFlex(canID, MotorType.kBrushless);

    config = new SparkFlexConfig();
    config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    ClosedLoopConfig closedLoop = new ClosedLoopConfig().pid(kP, kI, kD);
    config.apply(closedLoop);

    SoftLimitConfig softLimit =
        new SoftLimitConfig()
            .forwardSoftLimit(ConfigUtil.getConfigDouble("armForwardSoftLimit", 0))
            .forwardSoftLimitEnabled(ConfigUtil.getConfigBoolean("armSoftLimitEnabled", true))
            .reverseSoftLimit(ConfigUtil.getConfigDouble("armReverseSoftLimit", 0))
            .reverseSoftLimitEnabled(
                ConfigUtil.getConfigBoolean("armReverseSoftLimitEnabled", false));
    config.apply(softLimit);

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Set current limits
    config.smartCurrentLimit(statorCurrentLimit);

    // Closed Loop Controller
    controller = motor.getClosedLoopController();

    // Save configuration
    motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double voltage) {
    controller.setReference(voltage, ControlType.kVoltage);
  }

  // Encoder Values
  @Override
  public void setPosition(double position) {
    setPosition(
        position,
        ConfigUtil.getConfigDouble("defaultArmVel", 5),
        ConfigUtil.getConfigDouble("defaultArmAccel", 1));
  }

  public void setPosition(double position, double velocityRadPerSec) {
    setPosition(position, velocityRadPerSec, ConfigUtil.getConfigDouble("defaultArmAccel", 1));
  }

  public void setPosition(double position, double VelocityRadPerSec, double accelRadPerSecSquared) {
    feedforward.calculate(
        Units.degreesToRadians(position), VelocityRadPerSec, accelRadPerSecSquared);
    controller.setReference(position, ControlType.kPosition);
  }

  @Override
  public void setVelocity(double velocity) {
    controller.setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.currentVoltage = motor.getBusVoltage();
    inputs.position = encoder.getPosition();
    inputs.temperature = motor.getMotorTemperature();
    inputs.velocity = encoder.getVelocity();
    inputs.currentAmps = motor.getOutputCurrent();
  }
}
