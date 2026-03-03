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

package org.austincans.frc2158.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2158.Constants;
import org.austincans.frc2158.Robot;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2158.util.ConfigUtil;
import org.austincans.frc2158.util.LoggedTracer;
import org.austincans.frc2158.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module extends AdvancedSubsystem {
  private static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS");
  private static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV");
  private static final LoggedTunableNumber drivekT =
      new LoggedTunableNumber("Drive/Module/DrivekT");
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP");
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD");
  private static final LoggedTunableNumber turnkP = new LoggedTunableNumber("Drive/Module/TurnkP");
  private static final LoggedTunableNumber turnkD = new LoggedTunableNumber("Drive/Module/TurnkD");

  static {
    switch (Constants.getRobot()) {
      case COMPBOT, DEVBOT -> {
        drivekS.initDefault(5.0);
        drivekV.initDefault(0);
        // Multiplied by desired wheelTorqueNm
        drivekT.initDefault(DriveConstants.driveReduction / DCMotor.getKrakenX60Foc(1).KtNMPerAmp);
        drivekP.initDefault(ConfigUtil.getConfigDouble("DRIVE_P", 0.0));
        drivekD.initDefault(ConfigUtil.getConfigDouble("DRIVE_D", 0.0));
        turnkP.initDefault(ConfigUtil.getConfigDouble("TURN_P", 0.0));
        turnkD.initDefault(ConfigUtil.getConfigDouble("TURN_D", 0.0));
      }
      default -> {
        drivekS.initDefault(0.014);
        drivekV.initDefault(0.134);
        drivekT.initDefault(0);
        drivekP.initDefault(1);
        drivekD.initDefault(0);
        turnkP.initDefault(10.0);
        turnkD.initDefault(0);
      }
    }
  }

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
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
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;

    io.setDrivePID(drivekP.get(), 0, drivekD.get());
    io.setTurnPID(turnkP.get(), 0, turnkD.get());

    ffModel = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
    io.getHardware().forEach(e -> registerHardware(e));
  }

  public void periodic() {
    io.updateInputs(inputs);
    io.checkForFaults().forEach(this::addHardwareFault);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Update tunable numbers
    if (drivekS.hasChanged(hashCode()) || drivekV.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());
    }
    if (drivekP.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
      io.setDrivePID(drivekP.get(), 0, drivekD.get());
    }
    if (turnkP.hasChanged(hashCode()) || turnkD.hasChanged(hashCode())) {
      io.setTurnPID(turnkP.get(), 0, turnkD.get());
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(
        !driveMotorConnectedDebouncer.calculate(inputs.driveConnected) && !Robot.isJITing());
    turnDisconnectedAlert.set(
        !turnMotorConnectedDebouncer.calculate(inputs.turnConnected) && !Robot.isJITing());
    turnEncoderDisconnectedAlert.set(
        !turnEncoderConnectedDebouncer.calculate(inputs.turnEncoderConnected) && !Robot.isJITing());

    // Record cycle time
    LoggedTracer.record("Drive/Module" + index);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnAbsolutePosition);

    // Apply setpoints
    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.wheelRadius;
    io.setDriveVelocity(speedRadPerSec * ConfigUtil.getConfigDouble("SPEED_CONSTANT", 1.0));
    io.setTurnPosition(state.angle);
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
    io.setDriveVelocity(
        speedRadPerSec * ConfigUtil.getConfigDouble("SPEED_CONSTANT", 1.0),
        ffModel.calculate(speedRadPerSec) + wheelTorqueNm * drivekT.get());
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Runs the module with the specified output toward Angular Motions */
  public void runAngularCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d(constants.LocationX, constants.LocationY).plus(Rotation2d.kCCW_Pi_2));
}

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }

  /* Sets brake mode to {@code enabled} */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  stop();
                  io.setTurnPosition(new Rotation2d(20));
                },
                this),
            Commands.waitSeconds(0.3),
            Commands.runOnce(
                () -> {
                  if (Units.radiansToDegrees(inputs.turnVelocityRadPerSec) < 20) {
                    addSubsystemFault(
                        "SWRV" + String.valueOf(index),
                        "[System Check] Rotation motor encoder velocity measured too slow",
                        false);
                  }
                },
                this),
            Commands.run(
                    () -> {
                      io.setTurnPosition(Rotation2d.fromDegrees(90));
                    },
                    this)
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.turnAbsolutePosition.getDegrees() < 70
                      || inputs.turnAbsolutePosition.getDegrees() > 110) {
                    addSubsystemFault(
                        "SWRV" + String.valueOf(index),
                        "[System Check] Rotation Motor did not reach target position",
                        false);
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  io.setDriveVelocity(Units.degreesToRadians(90));
                },
                this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  if (Units.radiansToDegrees(inputs.driveVelocityRadPerSec) < 0.25) {
                    addSubsystemFault(
                        "SWRV" + String.valueOf(index),
                        "[System Check] Drive motor encoder velocity too slow",
                        false);
                  }
                  io.setDriveVelocity(0);
                },
                this),
            Commands.waitSeconds(0.25))
        .andThen(Commands.runOnce(this::stop, this));
  }
}
