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

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2026.subsystems.intake.IntakeConstants;
import org.austincans.frc2026.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2026.subsystems.selfcheck.SelfCheckingSparkFlex;
import org.austincans.frc2026.util.ConfigUtil;

public class DeployIOSparkFlex implements DeployIO {
  private final SparkFlex deployMotor1;
  private final SparkFlex deployMotor2;
  private final SelfCheckingSparkFlex deployMotorSelfCheck1;
  private final SelfCheckingSparkFlex deployMotorSelfCheck2;

  private final SparkFlexConfig config;
  private final RelativeEncoder deployEncoder;
  private final SparkClosedLoopController deployController;
  private boolean deployed = false;

  public DeployIOSparkFlex() {
    deployMotor1 = new SparkFlex(ConfigUtil.getConfigInt("INTAKE_DEPLOY_ID", 26), MotorType.kBrushless);
    deployMotor2 = new SparkFlex(ConfigUtil.getConfigInt("INTAKE_DEPLOY_ID", 27), MotorType.kBrushless);
    deployMotorSelfCheck1 = new SelfCheckingSparkFlex("Intake Deploy Motor 1", deployMotor1);
    deployMotorSelfCheck2 = new SelfCheckingSparkFlex("Intake Deploy Motor 2", deployMotor2);

    config = new SparkFlexConfig();
    config.idleMode(IntakeConstants.deployBreakEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    ClosedLoopConfig closedLoop = new ClosedLoopConfig().pid(IntakeConstants.kP, IntakeConstants.kI,
        IntakeConstants.kD);
    config.apply(closedLoop);

    // Configure deployEncoder
    deployEncoder = deployMotor1.getEncoder();
    deployEncoder.setPosition(0);

    deployController = deployMotor1.getClosedLoopController();

    // Save configuration
    deployMotor1.configureAsync(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    deployMotor2.configureAsync(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void deploy() {
    deployed = true;
    deployMotor1.set(0.05);
    deployMotor2.set(-0.05);
  }

  @Override
  public void retract() {
    deployed = false;
    deployMotor1.set(0.1);
    deployMotor2.set(-0.1);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isDeployed = deployed;
    inputs.deployMotorVoltage = deployMotor1.getBusVoltage();
    inputs.deployPositionRad = Units.rotationsToRadians(deployEncoder.getPosition());
    inputs.deployTemperatureC = deployMotor1.getMotorTemperature();
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(deployMotorSelfCheck1, deployMotorSelfCheck2);
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();

    List<HardwareFault> deployFault = deployMotorSelfCheck1.checkForFaults();
    deployFault.addAll(deployMotorSelfCheck2.checkForFaults());
    if (deployFault != null) {
      faults.addAll(deployFault);
    }

    if (faults.isEmpty()) {
      return List.of();
    } else {
      return faults;
    }
  }

  @Override
  public void stopMotors() {
    deployMotor1.stopMotor();
    deployMotor2.stopMotor();
  }
}
