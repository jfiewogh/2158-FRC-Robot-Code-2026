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
// Updated on Tuesday November 18, 2025 at 15:59:53 CST

package org.austincans.frc2158.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

/** Elevator subsystem using SparkFlex with Vortex motor */
public class Elevator extends AdvancedSubsystem {
  // Constants
  private final double gearRatio = 15;
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0;
  @Getter private final double minHeightMeters = 0;
  @Getter private final double maxHeightMeters = 2;
  private final double drumRadius = 0.0254; // meters

  // Feedforward
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          0, // kS
          0, // kG
          0, // kV
          0 // kA
          );

  // Control mode
  public enum ControlMode {
    OPEN_LOOP,
    POSITION,
    VELOCITY,
  }

  private ControlMode currentControlMode = ControlMode.OPEN_LOOP;
  private double targetPosition = 0.0;
  private double targetVelocity = 0.0;

  // Profiled PID Controller
  private ProfiledPIDController profiledPIDController;
  private TrapezoidProfile.Constraints constraints;

  // IO
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs;

  private ElevatorSim elevatorSim;

  /** Creates a new Elevator Subsystem. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    profiledPIDController = new ProfiledPIDController(kP, kI, kD, constraints);

    elevatorSim =
        new ElevatorSim(
            DCMotor.getNeoVortex(1), // Motor type
            gearRatio,
            5, // Carriage mass (kg)
            drumRadius, // Drum radius (m)
            minHeightMeters, // Min height (m)
            maxHeightMeters, // Max height (m)
            true, // Simulate gravity
            0 // Starting height (m)
            );
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param positionRadians The position (angle) setpoint. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param velocityRadPerSec The velocity setpoint.
   * @param accelRadPerSecSquared The acceleration setpoint.
   * @return The computed feedforward.
   */
  public double calculate(
      double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {

    return feedforward.getKs() * Math.signum(velocityRadPerSec)
        + feedforward.getKg() * Math.cos(positionRadians)
        + feedforward.getKv() * velocityRadPerSec
        + feedforward.getKa() * accelRadPerSecSquared;
  }

  /**
   * Control loop function that runs at a fixed frequency. This is used for SparkMAX and SparkFlex
   * controllers to implement closed-loop control outside of the main robot loop.
   */
  private void controlLoopFn() {
    switch (currentControlMode) {
      case POSITION:
        double currentPos = inputs.positionMeters;
        double output = profiledPIDController.calculate(currentPos, targetPosition);
        double velocity = profiledPIDController.getSetpoint().velocity;
        double feedforwardOutput = feedforward.calculate(velocity);
        setVoltage(output + feedforwardOutput);
        break;
      case VELOCITY:
        double currentVel = inputs.positionMeters;
        double velOutput = profiledPIDController.calculate(currentVel, targetVelocity);
        double accel = profiledPIDController.getSetpoint().velocity - currentVel;
        double velFeedforwardOutput =
            calculate(0, targetVelocity, accel); // No gravity compensation

        // Apply the combined PID output and feedforward to the motor
        double velocityVoltage = velOutput + velFeedforwardOutput;
        io.setVoltage(velocityVoltage);
        break;
      case OPEN_LOOP:
      default:
        // Do nothing, voltage is set directly
        break;
    }
  }

  /** Update simulation and telemetry. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    controlLoopFn();

    Logger.processInputs("Elevator", inputs);
  }

  /**
   * Set elevator position.
   *
   * @param position The target position in meters
   */
  public void setPosition(double position) {
    setPosition(position, 0);
  }

  /**
   * Set elevator position with acceleration.
   *
   * @param position The target position in meters
   * @param acceleration The acceleration in meters per second squared
   */
  public void setPosition(double position, double acceleration) {
    // Convert meters to rotations
    double positionRotations = position / (2.0 * Math.PI * drumRadius);

    // Use the ProfiledPIDController
    targetPosition = positionRotations;
    currentControlMode = ControlMode.POSITION;

    // If acceleration is specified, update constraints
    if (acceleration > 0) {
      double maxAccelRotations = acceleration / (2.0 * Math.PI * drumRadius);
      constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAccelRotations);
      profiledPIDController.setConstraints(constraints);
    }
  }

  /**
   * Set elevator velocity.
   *
   * @param velocity The target velocity in meters per second
   */
  public void setVelocity(double velocity) {
    setVelocity(velocity, 0);
  }

  /**
   * Set elevator velocity with acceleration.
   *
   * @param velocity The target velocity in meters per second
   * @param acceleration The acceleration in meters per second squared
   */
  public void setVelocity(double velocity, double acceleration) {
    // Convert meters/sec to rotations/sec
    double velocityRotations = velocity / (2.0 * Math.PI * drumRadius);

    // Use the ProfiledPIDController
    targetVelocity = velocityRotations;
    currentControlMode = ControlMode.VELOCITY;

    // If acceleration is specified, update constraints
    if (acceleration > 0) {
      double maxAccelRotations = acceleration / (2.0 * Math.PI * drumRadius);
      constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAccelRotations);
      profiledPIDController.setConstraints(constraints);
    }
  }

  /**
   * Set motor voltage directly.
   *
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    currentControlMode = ControlMode.OPEN_LOOP;
    io.setVoltage(voltage);
  }

  /**
   * Get the elevator simulation for testing.
   *
   * @return The elevator simulation model
   */
  public ElevatorSim getSimulation() {
    return elevatorSim;
  }

  /**
   * Creates a command to set the elevator to a specific height.
   *
   * @param heightMeters The target height in meters
   * @return A command that sets the elevator to the specified height
   */
  public Command setHeightCommand(double heightMeters) {
    return runOnce(() -> setPosition(heightMeters));
  }

  /**
   * Creates a command to move the elevator to a specific height with a profile.
   *
   * @param heightMeters The target height in meters
   * @return A command that moves the elevator to the specified height
   */
  public Command moveToHeightCommand(double heightMeters) {
    return run(() -> {
          // Just set the position and let the profiled controller handle it
          setPosition(heightMeters);
        })
        .until(
            () -> {
              double currentHeight = inputs.velocityMetersPerSecond * (2.0 * Math.PI * drumRadius);
              return Math.abs(heightMeters - currentHeight) < 0.02; // 2cm tolerance
            });
  }

  /**
   * Creates a command to stop the elevator.
   *
   * @return A command that stops the elevator
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the elevator at a specific velocity.
   *
   * @param velocityMetersPerSecond The target velocity in meters per second
   * @return A command that moves the elevator at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityMetersPerSecond) {
    return run(() -> setVelocity(velocityMetersPerSecond));
  }

  @Override
  public Command systemCheckCommand() {
    // TODO Auto-generated method stub
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  setVelocity(0);
                  setHeightCommand(0.5);
                },
                this),
            Commands.waitSeconds(0.3),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityMetersPerSecond < 1) {
                    addFault(
                        "[System Check] Rotation motor encoder velocity measured too slow",
                        false,
                        true);
                  }
                },
                this),
            Commands.run(
                    () -> {
                      setPosition(1);
                    },
                    this)
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.positionMeters < 0.95 || inputs.positionMeters > 1.05) {
                    addFault(
                        "[System Check] Rotation Motor did not reach target position", false, true);
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  moveAtVelocityCommand(1);
                },
                this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityMetersPerSecond < 0.95) {
                    addFault("[System Check] Drive motor encoder velocity too slow", false, true);
                  }
                  moveAtVelocityCommand(0);
                },
                this),
            Commands.waitSeconds(0.25))
        .until(() -> getFaults().size() > 0)
        .andThen(Commands.runOnce(this::stopCommand, this));
  }
}
