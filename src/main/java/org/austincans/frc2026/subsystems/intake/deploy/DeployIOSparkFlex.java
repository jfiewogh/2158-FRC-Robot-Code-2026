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

package org.austincans.frc2026.subsystems.intake.deploy;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2026.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2026.subsystems.selfcheck.SelfCheckingSparkFlex;

public class DeployIOSparkFlex implements DeployIO {
  private final SparkFlex leftDeployMotor;
  private final SparkFlex rightDeployMotor;
  private final SelfCheckingSparkFlex leftDeployMotorSelfCheck;
  private final SelfCheckingSparkFlex rightDeployMotorSelfCheck;

  private final RelativeEncoder rightDeployEncoder;
  private final RelativeEncoder leftDeployEncoder;
  private final SparkClosedLoopController rightDeployController;
  private final SparkClosedLoopController leftDeployController;
  private boolean deployed = false;

  public DeployIOSparkFlex() {
    leftDeployMotor = new SparkFlex(26, MotorType.kBrushless);
    rightDeployMotor = new SparkFlex(27, MotorType.kBrushless);
    leftDeployMotorSelfCheck = new SelfCheckingSparkFlex("Intake Deploy Motor 1", leftDeployMotor);
    rightDeployMotorSelfCheck =
        new SelfCheckingSparkFlex("Intake Deploy Motor 2", rightDeployMotor);

    // SparkFlexConfig rightConfig = new SparkFlexConfig();
    // rightConfig.idleMode(IntakeConstants.deployBrakeEnabled ? IdleMode.kBrake :
    // IdleMode.kCoast);
    // rightConfig.softLimit.forwardSoftLimit(7.8);
    // rightConfig.softLimit.reverseSoftLimit(0);
    // rightConfig.closedLoop.pid(0, 0, 0);
    // rightConfig.inverted(true);
    // rightDeployMotor.configureAsync(
    // rightConfig, ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters);

    // SparkFlexConfig leftConfig = new SparkFlexConfig();
    // leftConfig.idleMode(IntakeConstants.deployBrakeEnabled ? IdleMode.kBrake :
    // IdleMode.kCoast);
    // leftConfig.softLimit.forwardSoftLimit(7.8);
    // leftConfig.softLimit.reverseSoftLimit(0);
    // leftConfig.closedLoop.pid(0, 0, 0);
    // leftConfig.follow(rightDeployMotor);
    // leftDeployMotor.configureAsync(
    // leftConfig, ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters);
    leftDeployEncoder = leftDeployMotor.getEncoder();

    rightDeployController = rightDeployMotor.getClosedLoopController();
    rightDeployEncoder = rightDeployMotor.getEncoder();
    leftDeployController = leftDeployMotor.getClosedLoopController();
  }

  public void setVelocity(double velocityRotationsPerSec) {
    rightDeployMotor.set(velocityRotationsPerSec);
    leftDeployMotor.set(velocityRotationsPerSec);
    // rightDeployController.setSetpoint(velocityRotationsPerSec,
    // ControlType.kVelocity);
  }

  @Override
  public void deploy() {
    System.out.println("Deploy");
    leftDeployController.setSetpoint(0.15, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    rightDeployController.setSetpoint(0.15, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    // leftDeployMotor.set(0.3);
    // rightDeployMotor.set(0.3);
  }

  @Override
  public void retract() {
    System.out.println("Retract");
    // if (leftDeployEncoder.getPosition() > 2) {
    leftDeployController.setSetpoint(-0.15, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    rightDeployController.setSetpoint(-0.15, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    // }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isDeployed = deployed;
    inputs.deployMotorVoltage = rightDeployMotor.getBusVoltage();
    inputs.deployPositionRad = Units.rotationsToRadians(rightDeployEncoder.getPosition());
    inputs.deployTemperatureC = rightDeployMotor.getMotorTemperature();
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(leftDeployMotorSelfCheck, rightDeployMotorSelfCheck);
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();
    List<HardwareFault> deployFault = leftDeployMotorSelfCheck.checkForFaults();
    deployFault.addAll(rightDeployMotorSelfCheck.checkForFaults());
    if (deployFault != null) {
      faults.addAll(deployFault);
    }
    return faults;
  }

  @Override
  public void stopMotors() {
    leftDeployMotor.stopMotor();
    rightDeployMotor.stopMotor();
  }
}
