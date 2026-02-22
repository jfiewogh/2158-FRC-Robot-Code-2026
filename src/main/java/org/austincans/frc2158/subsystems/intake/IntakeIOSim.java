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
// Updated on Sunday February 22, 2026 at 14:08:53 CST

package org.austincans.frc2158.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.austincans.frc2158.Constants;
import org.austincans.frc2158.math.Angle;
import org.austincans.frc2158.subsystems.sim.IntakeSim;
import org.ironmaple.simulation.IntakeSimulation;

public class IntakeIOSim implements IntakeIO {
  private final IntakeSim intakeSim;

  private final DCMotor deployMotor = DCMotor.getNeoVortex(1);
  private final DCMotor rollerMotor = DCMotor.getNeoVortex(1);

  private final DCMotorSim deploySim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              deployMotor,
              IntakeConstants.deployRotationalInertiaMetersSq,
              IntakeConstants.deployGearing),
          deployMotor);
  private final DCMotorSim rollerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              rollerMotor,
              IntakeConstants.intakeRotationalInertiaMetersSq,
              IntakeConstants.intakeGearing),
          rollerMotor);

  private double intakeAppliedVolts;

  public IntakeIOSim(IntakeSimulation intakeSimulation) {
    intakeSim = new IntakeSim(intakeSimulation);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.deployPositionRad = deploySim.getAngularPositionRad();

    double deployAppliedVolts =
        intakePIDController.calculate(inputs.deployPositionRad)
            / (Math.PI * 2)
            * Constants.maxVoltage;

    deploySim.setInputVoltage(deployAppliedVolts);
    deploySim.update(Constants.loopPeriodSecs);

    rollerSim.setInputVoltage(intakeAppliedVolts);
    rollerSim.update(Constants.loopPeriodSecs);

    inputs.deployMotorVoltage = deploySim.getInputVoltage();
    inputs.deployTemperatureC = 0;
    inputs.intakeMotorVoltage = rollerSim.getInputVoltage();
    inputs.intakeVelocityRadPerSec = rollerSim.getAngularVelocity().in(RadiansPerSecond);
    inputs.intakeTemperatureC = 0;
    // inputs.gamePiecesCount = intakeSim.getGamePiecesCount();

    intakeSim.update(inputs.intakeVelocityRadPerSec);
  }

  @Override
  public void setRelativeVelocity(double velocity) {
    intakeAppliedVolts = velocity * Constants.maxVoltage;
  }

  @Override
  public void setVoltage(double voltage) {
    intakeAppliedVolts = voltage;
  }

  @Override
  public void setDeployAngle(Angle angle) {
    System.out.println(angle.toRadians());
    intakePIDController.setGoal(angle.toRadians());
  }
}
