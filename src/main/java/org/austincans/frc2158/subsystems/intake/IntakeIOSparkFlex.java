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
// Updated on Thursday February 12, 2026 at 19:07:37 CST

package org.austincans.frc2158.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2158.math.Angle;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingSparkFlex;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;
import org.austincans.frc2158.util.ConfigUtil;

/** Add your docs here. */
public class IntakeIOSparkFlex implements IntakeIO {
  private final SparkFlex deployMotor;
  private final SparkFlex intakeMotor;
  private final SelfCheckingSparkFlex deployMotorSelfCheck;
  private final SelfCheckingSparkFlex intakeMotorSelfCheck;

  private final SparkFlexConfig config;
  private final RelativeEncoder deployEncoder;
  private final SparkClosedLoopController deployController;

  public IntakeIOSparkFlex() {
    // Initialize motor controllers
    deployMotor =
        new SparkFlex(ConfigUtil.getConfigInt("intakeDeployMotorID1", 0), MotorType.kBrushless);
    intakeMotor = new SparkFlex(ConfigUtil.getConfigInt("intakeMotorID1", 0), MotorType.kBrushless);

    deployMotorSelfCheck = new SelfCheckingSparkFlex("Intake Deploy Motor", deployMotor);
    intakeMotorSelfCheck = new SelfCheckingSparkFlex("Intake Motor", intakeMotor);

    config = new SparkFlexConfig();
    config.idleMode(deployBreakEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    ClosedLoopConfig closedLoop = new ClosedLoopConfig().pid(kP, kI, kD);
    config.apply(closedLoop);

    // Configure deployEncoder
    deployEncoder = deployMotor.getEncoder();
    deployEncoder.setPosition(0);

    deployController = deployMotor.getClosedLoopController();

    // Save configuration
    deployMotor.configureAsync(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.deployMotorVoltage = deployMotor.getBusVoltage();
    inputs.deployPositionRad = Units.rotationsToRadians(deployEncoder.getPosition());
    inputs.deployTemperatureC = deployMotor.getMotorTemperature();
    inputs.intakeMotorVoltage = intakeMotor.getBusVoltage();
    inputs.intakeTemperatureC = intakeMotor.getMotorTemperature();
  }

  @Override
  public void setDeployAngle(Angle angle) {
    deployController.setSetpoint(angle.toRotations(), ControlType.kPosition);
  }

  @Override
  public void setRelativeVelocity(double velocity) {
    intakeMotor.set(velocity);
  }

  @Override
  public void setVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    List<SubsystemFault> deployFault = deployMotorSelfCheck.checkForFaults();
    if (deployFault != null) {
      faults.addAll(deployFault);
    }

    List<SubsystemFault> intakeFault = intakeMotorSelfCheck.checkForFaults();
    if (intakeFault != null) {
      faults.addAll(intakeFault);
    }

    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }
}
