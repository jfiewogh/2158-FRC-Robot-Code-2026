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

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2026.subsystems.intake.IntakeConstants;
import org.austincans.frc2026.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2026.subsystems.selfcheck.SelfCheckingTalonFXS;

public class DeployIOTalonFXS implements DeployIO {
  private final TalonFXS deployMotor1;
  private final TalonFXS deployMotor2;
  private final SelfCheckingTalonFXS deploy1MotorSelfCheck;
  private final SelfCheckingTalonFXS deploy2MotorSelfCheck;
  private boolean deployed = false;

  // private final TalonFXSConfiguration config;

  // private final DigitalInput magLimitSwitch =
  //     new DigitalInput(IntakeConstants.magLimitSwitchChannel);

  // private final CANcoder deployEncoder;

  public DeployIOTalonFXS() {
    // deployEncoder = new CANcoder(ConfigUtil.getConfigInt("DEPLOY_ENCODER_ID",
    // 0));
    deployMotor1 = new TalonFXS(IntakeConstants.deployCanID1, IntakeConstants.intakeCANBus);
    deployMotor2 = new TalonFXS(IntakeConstants.deployCanID2, IntakeConstants.intakeCANBus);
    deploy1MotorSelfCheck = new SelfCheckingTalonFXS("Intake Deploy Motor 1", deployMotor1);
    deploy2MotorSelfCheck = new SelfCheckingTalonFXS("Intake Deploy Motor 2", deployMotor2);

    // config = new TalonFXSConfiguration();
    // config.MotorOutput.NeutralMode =
    //     IntakeConstants.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    // config.Slot0.kP = IntakeConstants.kP;
    // config.Slot0.kI = IntakeConstants.kI;
    // config.Slot0.kD = IntakeConstants.kD;
    // config.Slot0.kS = IntakeConstants.kS;
    // config.Slot0.kG = IntakeConstants.kG;
    // config.Slot0.kV = IntakeConstants.kV;
    // config.Slot0.kA = IntakeConstants.kA;

    // config.ExternalFeedback.FeedbackRemoteSensorID = deployEncoder.getDeviceID();
    // config.ExternalFeedback.ExternalFeedbackSensorSource =
    // ExternalFeedbackSensorSourceValue.FusedCANcoder;
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // deployMotor1.getConfigurator().apply(config);
    // config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // deployMotor2.getConfigurator().apply(config);
    // deployMotor2.setControl(
    //     new Follower(IntakeConstants.deployCanID1, MotorAlignmentValue.Aligned));
    // deployEncoder.setPosition(0);
  }

  @Override
  public void stopMotors() {
    deployMotor1.stopMotor();
  }

  @Override
  public void setVelocity(double velocityRotationsPerSec) {}
  ;

  @Override
  public void deploy() {
    deployed = true;
    System.out.println("DEPLOY");
    deployMotor1.set(0.05); // TODO: CHANGE ME
    deployMotor2.set(-0.05);
  }

  @Override
  public void retract() {
    deployed = false;
    System.out.println("RETRACT");
    deployMotor1.set(-0.05);
    deployMotor2.set(0.05);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isDeployed = deployed;
    inputs.deployMotorVoltage = deployMotor1.getMotorVoltage().getValueAsDouble();
    inputs.deployPositionRad = deployMotor1.getPosition().getValue().in(Units.Radians);
    inputs.deployTemperatureC = deployMotor1.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();
    List<HardwareFault> deployFault = deploy2MotorSelfCheck.checkForFaults();
    deployFault.addAll(deploy1MotorSelfCheck.checkForFaults());
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
  public List<SelfChecking> getHardware() {
    return List.of(deploy1MotorSelfCheck, deploy2MotorSelfCheck);
  }
}
