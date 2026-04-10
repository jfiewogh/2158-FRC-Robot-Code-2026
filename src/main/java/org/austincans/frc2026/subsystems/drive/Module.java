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

package org.austincans.frc2026.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.austincans.frc2026.subsystems.drive.ModuleIO.ModuleIOOutputMode;
import org.austincans.frc2026.subsystems.drive.ModuleIO.ModuleIOOutputs;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final ModuleIOOutputs outputs = new ModuleIOOutputs();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private SimpleMotorFeedforward ffModel;

  // Connected debouncers
  private final Debouncer driveMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Connection alerts
  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert encoderDisconnectedAlert;

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    encoderDisconnectedAlert =
        new Alert(
            "Disconnected encoder on module " + Integer.toString(index) + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Record cycle times
    // LoggedTracer.record("Drive/Module" + index + "/Periodic");
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
    // LoggedTracer.record("Drive/Module" + index + "/AfterScheduler");
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.wheelRadius;
    outputs.mode = ModuleIOOutputMode.DRIVE;
    outputs.driveVelocityRadPerSec = speedRadPerSec;
    outputs.turnRotation = state.angle;
    outputs.turnNeutral = Math.abs(state.angle.minus(getAngle()).getDegrees()) < 1;
  }

  /**
   * Runs the module with the specified setpoint state and a setpoint wheel force used for
   * torque-based feedforward.
   */
  public void runSetpoint(SwerveModuleState state, double wheelTorqueNm) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnAbsolutePosition);

    // Apply setpoints
    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.wheelRadius;
    outputs.mode = ModuleIOOutputMode.DRIVE;
    outputs.driveVelocityRadPerSec = speedRadPerSec;
    outputs.driveFeedforward = ffModel.calculate(speedRadPerSec) + wheelTorqueNm;
    outputs.turnRotation = state.angle;
    outputs.turnNeutral =
        Math.abs(state.angle.minus(getAngle()).getDegrees()) < DriveConstants.turnDeadbandDeg;
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    outputs.mode = ModuleIOOutputMode.CHARACTERIZE;
    outputs.driveCharacterizationOutput = output;
    outputs.turnRotation = Rotation2d.kZero;
  }

  /** Runs the module with the specified output toward Angular Motions */
  public void runAngularCharacterization(double output) {
    outputs.mode = ModuleIOOutputMode.CHARACTERIZE;
    outputs.driveCharacterizationOutput = output;
    outputs.turnRotation = Rotation2d.kCCW_Pi_2;
  }

  /** Disables all motor outputs in brake mode. */
  public void brake() {
    outputs.mode = ModuleIOOutputMode.BRAKE;
  }

  /** Disables all motor outputs in coast mode. */
  public void coast() {
    outputs.mode = ModuleIOOutputMode.COAST;
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
