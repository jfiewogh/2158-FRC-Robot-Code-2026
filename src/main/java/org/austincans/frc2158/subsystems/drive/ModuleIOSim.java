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
// Updated on Wednesday December 24, 2025 at 21:41:23 CST

package org.austincans.frc2158.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;

  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(0, 0, 0);
  private PIDController turnController = new PIDController(0, 0, 0);
  private double driveFFVolts = 0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;

    // configures a generic motor controller for drive motor
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Units.Amps.of(DriveConstants.driveCurrentLimitAmps));
    this.turnMotor =
        moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Units.Amps.of(DriveConstants.turnCurrentLimitAmps));

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(getDriveWheelFinalSpeedRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts =
          turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveMotor.requestVoltage(Units.Volts.of(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0)));
    turnMotor.requestVoltage(Units.Volts.of(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0)));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Units.Radians);
    inputs.driveVelocityRadPerSec = getDriveWheelFinalSpeedRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = moduleSimulation.getDriveMotorSupplyCurrent().in(Units.Amps);
    inputs.driveStatorCurrentAmps = moduleSimulation.getDriveMotorStatorCurrent().in(Units.Amps);
    inputs.driveTorqueCurrentAmps = 0.0;

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = getSteerAbsoluteFacing();
    inputs.turnPosition = inputs.turnAbsolutePosition;
    inputs.turnVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(Units.RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnSupplyCurrentAmps = moduleSimulation.getSteerMotorSupplyCurrent().in(Units.Amps);
    inputs.turnStatorCurrentAmps = moduleSimulation.getSteerMotorSupplyCurrent().in(Units.Amps);
    inputs.turnTorqueCurrentAmps = 0.0;

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
    driveClosedLoop = true;
    driveFFVolts = feedforward;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveController.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnController.setPID(kP, kI, kD);
  }

  private Rotation2d getSteerAbsoluteFacing() {
    return moduleSimulation.getSteerAbsoluteFacing();
  }

  private double getDriveWheelFinalSpeedRadPerSec() {
    return moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond);
  }
}
