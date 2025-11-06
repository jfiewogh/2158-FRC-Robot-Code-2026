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
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

package org.austincans.frc2158.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

/** Elevator subsystem using SparkFlex with Vortex motor */
@Logged
public class ElevatorSubsystem extends SubsystemBase {

  // Constants
  private final int canID = 1;
  private final double gearRatio = 15;
  private final double kP = 1;
  private final double kI = 0;
  private final double kD = 0;
  private final double maxVelocity = 1; // meters per second
  private final double maxAcceleration = 1; // meters per second squared
  private final boolean brakeMode = true;
  @Getter private final double minHeightMeters = 0;
  @Getter private final double maxHeightMeters = 2;

  @SuppressWarnings("unused")
  private final boolean enableStatorLimit = true;

  private final double statorCurrentLimit = 40;

  @SuppressWarnings("unused")
  private final boolean enableSupplyLimit = false;

  @SuppressWarnings("unused")
  private final double supplyCurrentLimit = 40;

  private final double drumRadius = 0.0254; // meters

  // Feedforward
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          0, // kS
          0, // kG
          0, // kV
          0 // kA
          );

  // Motor controller
  private final SparkFlex motor;
  private final RelativeEncoder encoder;

  @SuppressWarnings("unused")
  private final SparkClosedLoopController pidController;

  private final SparkFlexConfig config;

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

  // Simulation
  private final ElevatorSim elevatorSim;

  /** Creates a new Elevator Subsystem. */
  public ElevatorSubsystem() {
    // Initialize motor controller
    motor = new SparkFlex(canID, MotorType.kBrushless);
    config = new SparkFlexConfig();
    config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    // Configure encoder
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // Configure PID controller
    pidController = motor.getClosedLoopController();
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);

    // Set ramp rates

    // Set current limits
    config.smartCurrentLimit((int) statorCurrentLimit);

    // Save configuration
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize simulation
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

    // Initialize ProfiledPIDController
    // Convert from meters to rotations for constraints
    double maxVelocityRotations = maxVelocity / (2.0 * Math.PI * drumRadius);
    double maxAccelerationRotations = maxAcceleration / (2.0 * Math.PI * drumRadius);

    constraints = new TrapezoidProfile.Constraints(maxVelocityRotations, maxAccelerationRotations);
    profiledPIDController = new ProfiledPIDController(kP, kI, kD, constraints);
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
        double currentPos = getPosition();
        double output = profiledPIDController.calculate(currentPos, targetPosition);
        double velocity = profiledPIDController.getSetpoint().velocity;
        double feedforwardOutput = feedforward.calculate(velocity);
        setVoltage(output + feedforwardOutput);
        break;
      case VELOCITY:
        double currentVel = getVelocity();
        double velOutput = profiledPIDController.calculate(currentVel, targetVelocity);
        double accel = profiledPIDController.getSetpoint().velocity - currentVel;
        double velFeedforwardOutput =
            calculate(0, targetVelocity, accel); // No gravity compensation

        // Apply the combined PID output and feedforward to the motor
        double velocityVoltage = velOutput + velFeedforwardOutput;
        motor.setVoltage(velocityVoltage);
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
    controlLoopFn();
  }

  /** Update simulation. */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    elevatorSim.setInput(getVoltage());

    // Update simulation by 20ms
    elevatorSim.update(0.020);

    // Convert meters to motor rotations
    double positionToRotations = (1 / (2.0 * Math.PI * drumRadius)) * gearRatio;
    @SuppressWarnings("unused")
    double motorPosition = elevatorSim.getPositionMeters() * positionToRotations;
    @SuppressWarnings("unused")
    double motorVelocity = elevatorSim.getVelocityMetersPerSecond() * positionToRotations;
  }

  /**
   * Get the current position in the selected unit.
   *
   * @return Position in selected unit
   */
  @Logged(name = "Position")
  public double getPosition() {
    // Rotations
    return encoder.getPosition() / gearRatio;
  }

  /**
   * Get the current velocity in rotations per second.
   *
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return encoder.getVelocity() / gearRatio / 60.0; // Convert from RPM to RPS
  }

  /**
   * Get the current applied voltage.
   *
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  /**
   * Get the current motor current.
   *
   * @return Motor current in amps
   */
  @Logged(name = "Current")
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Get the current motor temperature.
   *
   * @return Motor temperature in Celsius
   */
  @Logged(name = "Temperature")
  public double getTemperature() {
    return motor.getMotorTemperature();
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
    motor.setVoltage(voltage);
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
              double currentHeight = getPosition() * (2.0 * Math.PI * drumRadius);
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
}
