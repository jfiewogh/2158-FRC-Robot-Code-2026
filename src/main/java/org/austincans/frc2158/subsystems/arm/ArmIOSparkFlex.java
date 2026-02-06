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

package org.austincans.frc2158.subsystems.arm;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingSparkFlex;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;

public class ArmIOSparkFlex implements ArmIO {
  /* Motor */
  private final SparkFlex motor;
  private SelfCheckingSparkFlex selfCheckingMotor;
  private final RelativeEncoder encoder;
  private SparkFlexConfig config;
  private final SparkClosedLoopController controller;

  private int canID = ArmConstants.armCANID;
  private int statorCurrentLimit = ArmConstants.armCurrentLimit;
  private boolean brakeMode = ArmConstants.armIsBrakeMode;

  public ArmIOSparkFlex() {
    // Initialize motor controller
    motor = new SparkFlex(canID, MotorType.kBrushless);
    selfCheckingMotor = new SelfCheckingSparkFlex("Arm", motor);

    config = new SparkFlexConfig();
    config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    ClosedLoopConfig closedLoop =
        new ClosedLoopConfig().pid(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    config.apply(closedLoop);

    SoftLimitConfig softLimit =
        new SoftLimitConfig()
            .forwardSoftLimit(ArmConstants.armForwardSoftLimit)
            .forwardSoftLimitEnabled(ArmConstants.armSoftLimitEnabled)
            .reverseSoftLimit(ArmConstants.armReverseSoftLimit)
            .reverseSoftLimitEnabled(ArmConstants.armReverseSoftLimitEnabled);
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
    controller.setSetpoint(voltage, ControlType.kVoltage);
  }

  // Encoder Values
  @Override
  public void setPosition(double position) {
    setPosition(position, ArmConstants.defaultArmVel);
  }

  public void setPosition(double position, double VelocityRadPerSec) {
    feedforward.calculate(Units.degreesToRadians(position), VelocityRadPerSec);
    controller.setSetpoint(position, ControlType.kPosition);
  }

  public void setVelocity(double velocity) {
    controller.setSetpoint(velocity, ControlType.kVelocity);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.currentVoltage = motor.getBusVoltage();
    inputs.position = encoder.getPosition();
    inputs.temperature = motor.getMotorTemperature();
    inputs.velocity = encoder.getVelocity();
    inputs.currentAmps = motor.getOutputCurrent();
  }

  public void stopMotor() {
    setVoltage(0);
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = selfCheckingMotor.checkForFaults();

    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }
}
