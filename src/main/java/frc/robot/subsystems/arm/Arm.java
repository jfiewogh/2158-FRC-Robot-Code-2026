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

package frc.robot.subsystems.arm;

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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Arm subsystem using SparkFlex with Vortex motor */
@Logged
public class Arm extends SubsystemBase {

    // Constants
    private final int canID = 1;
    private final double gearRatio = 15;
    private final double kP = 1;
    private final double kI = 0;
    private final double kD = 0;
    private final double maxVelocity = 1; // rad/s
    private final double maxAcceleration = 1; // rad/s²
    private final boolean brakeMode = true;

    @SuppressWarnings("unused")
    private final boolean enableStatorLimit = true;

    private final double statorCurrentLimit = 40;

    @SuppressWarnings("unused")
    private final boolean enableSupplyLimit = false;

    @SuppressWarnings("unused")
    private final double supplyCurrentLimit = 40;

    private final double armLength = 1; // meters

    // Feedforward
    private final ArmFeedforward feedforward = new ArmFeedforward(
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

    // Telemetry
    private final NetworkTable telemetryTable;
    private NetworkTableEntry positionEntry;
    private NetworkTableEntry velocityEntry;
    private NetworkTableEntry voltageEntry;
    private NetworkTableEntry currentEntry;
    private NetworkTableEntry temperatureEntry;

    // Simulation
    private final SingleJointedArmSim armSim;

    /** Creates a new Arm Subsystem. */
    public Arm() {
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

        // Initialize telemetry
        telemetryTable = NetworkTableInstance.getDefault().getTable("");

        positionEntry = telemetryTable.getEntry("Position");
        velocityEntry = telemetryTable.getEntry("Velocity");

        voltageEntry = telemetryTable.getEntry("Voltage");
        currentEntry = telemetryTable.getEntry("Current");
        temperatureEntry = telemetryTable.getEntry("Temperature");

        // Initialize simulation
        armSim = new SingleJointedArmSim(
                DCMotor.getNeoVortex(1), // Motor type
                gearRatio,
                SingleJointedArmSim.estimateMOI(armLength, 5), // Arm moment of inertia
                armLength, // Arm length (m)
                Units.degreesToRadians(0), // Min angle (rad)
                Units.degreesToRadians(1.5707963267948966), // Max angle (rad)
                true, // Simulate gravity
                Units.degreesToRadians(0) // Starting position (rad)
                );

        // Initialize ProfiledPIDController
        // Convert from radians to rotations for constraints
        double maxVelocityRotations = maxVelocity / (2.0 * Math.PI);
        double maxAccelerationRotations = maxAcceleration / (2.0 * Math.PI);

        constraints = new TrapezoidProfile.Constraints(maxVelocityRotations, maxAccelerationRotations);
        profiledPIDController = new ProfiledPIDController(kP, kI, kD, constraints);
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param positionRadians The position (angle) setpoint. This angle should be measured from the horizontal (i.e. if
     *     the provided angle is 0, the arm should be parallel with the floor). If your encoder does not follow this
     *     convention, an offset should be added.
     * @param velocityRadPerSec The velocity setpoint.
     * @param accelRadPerSecSquared The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {

        return feedforward.getKs() * Math.signum(velocityRadPerSec)
                + feedforward.getKg() * Math.cos(positionRadians)
                + feedforward.getKv() * velocityRadPerSec
                + feedforward.getKa() * accelRadPerSecSquared;
    }

    /**
     * Control loop function that runs at a fixed frequency. This is used for SparkMAX and SparkFlex controllers to
     * implement closed-loop control outside of the main robot loop.
     */
    private void controlLoopFn() {
        switch (currentControlMode) {
            case POSITION:
                double currentPos = getPosition();
                double output = profiledPIDController.calculate(currentPos, targetPosition);
                double velocity = profiledPIDController.getSetpoint().velocity;
                double angle = getPositionRadians();
                double feedforwardOutput = feedforward.calculate(angle, velocity);
                setVoltage(output + feedforwardOutput);
                break;
            case VELOCITY:
                double currentVel = getVelocity();
                double velOutput = profiledPIDController.calculate(currentVel, targetVelocity);
                double accel = profiledPIDController.getSetpoint().velocity - currentVel;
                double armAngle = getPositionRadians();
                double velFeedforwardOutput = calculate(armAngle, targetVelocity, accel);

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
        updateTelemetry();
        controlLoopFn();
    }

    /** Update simulation. */
    @Override
    public void simulationPeriodic() {
        // Set input voltage from motor controller to simulation
        armSim.setInput(getVoltage());

        // Update simulation by 20ms
        armSim.update(0.020);
    }

    /** Update telemetry values. */
    private void updateTelemetry() {
        // Convert based on selected position unit
        switch ("Rotations") {
            case "Degrees":
                positionEntry.setDouble(Units.radiansToDegrees(getPositionRadians()));
                break;
            case "Radians":
                positionEntry.setDouble(getPositionRadians());
                break;
            case "Rotations":
            default:
                positionEntry.setDouble(getPosition());
                break;
        }

        velocityEntry.setDouble(getVelocity());

        voltageEntry.setDouble(getVoltage());

        currentEntry.setDouble(getCurrent());

        temperatureEntry.setDouble(getTemperature());
    }

    /**
     * Get the current position in rotations.
     *
     * @return Position in rotations
     */
    public double getPosition() {
        return encoder.getPosition() / gearRatio;
    }

    /**
     * Get the current position in radians.
     *
     * @return Position in radians
     */
    public double getPositionRadians() {
        double rotations = getPosition();
        return rotations * 2.0 * Math.PI;
    }

    /**
     * Get the current velocity in rotations per second.
     *
     * @return Velocity in rotations per second
     */
    public double getVelocity() {
        return encoder.getVelocity() / gearRatio / 60.0; // Convert from RPM to RPS
    }

    /**
     * Get the current applied voltage.
     *
     * @return Applied voltage
     */
    public double getVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    /**
     * Get the current motor current.
     *
     * @return Motor current in amps
     */
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    /**
     * Get the current motor temperature.
     *
     * @return Motor temperature in Celsius
     */
    public double getTemperature() {
        return motor.getMotorTemperature();
    }

    /**
     * Set arm angle.
     *
     * @param angleDegrees The target angle in degrees
     */
    public void setAngle(double angleDegrees) {
        setAngle(angleDegrees, 0);
    }

    /**
     * Set arm angle with acceleration.
     *
     * @param angleDegrees The target angle in degrees
     * @param acceleration The acceleration in rad/s²
     */
    public void setAngle(double angleDegrees, double acceleration) {
        // Convert degrees to rotations
        double angleRadians = Units.degreesToRadians(angleDegrees);
        double positionRotations = angleRadians / (2.0 * Math.PI);

        // Use the ProfiledPIDController
        targetPosition = positionRotations;
        currentControlMode = ControlMode.POSITION;

        // If acceleration is specified, update constraints
        if (acceleration > 0) {
            double maxAccelRotations = acceleration / (2.0 * Math.PI);
            constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAccelRotations);
            profiledPIDController.setConstraints(constraints);
        }
    }

    /**
     * Set arm angular velocity.
     *
     * @param velocityDegPerSec The target velocity in degrees per second
     */
    public void setVelocity(double velocityDegPerSec) {
        setVelocity(velocityDegPerSec, 0);
    }

    /**
     * Set arm angular velocity with acceleration.
     *
     * @param velocityDegPerSec The target velocity in degrees per second
     * @param acceleration The acceleration in degrees per second squared
     */
    public void setVelocity(double velocityDegPerSec, double acceleration) {
        // Convert degrees/sec to rotations/sec
        double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
        double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

        // Use the ProfiledPIDController
        targetVelocity = velocityRotations;
        currentControlMode = ControlMode.VELOCITY;

        // If acceleration is specified, update constraints
        if (acceleration > 0) {
            double maxAccelRotations = Units.degreesToRadians(acceleration) / (2.0 * Math.PI);
            constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAccelRotations);
            profiledPIDController.setConstraints(constraints);
        }

        // Apply velocity directly to the motor controller as well
        // This ensures immediate response while the control loop refines it
        double armAngle = getPositionRadians();
        double ffVolts = feedforward.calculateWithVelocities(armAngle, velocityRotations, 0);
        motor.setVoltage(ffVolts);
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
     * Get the arm simulation for testing.
     *
     * @return The arm simulation model
     */
    public SingleJointedArmSim getSimulation() {
        return armSim;
    }

    /**
     * Creates a command to set the arm to a specific angle.
     *
     * @param angleDegrees The target angle in degrees
     * @return A command that sets the arm to the specified angle
     */
    public Command setAngleCommand(double angleDegrees) {
        return runOnce(() -> setAngle(angleDegrees));
    }

    /**
     * Creates a command to move the arm to a specific angle with a profile.
     *
     * @param angleDegrees The target angle in degrees
     * @return A command that moves the arm to the specified angle
     */
    public Command moveToAngleCommand(double angleDegrees) {
        return run(() -> {
                    // Just set the position and let the profiled controller handle it
                    setAngle(angleDegrees);
                })
                .until(() -> {
                    double currentAngle = Units.radiansToDegrees(getPositionRadians());
                    return Math.abs(angleDegrees - currentAngle) < 2.0; // 2 degree tolerance
                })
                .finallyDo(interrupted -> setVelocity(0));
    }

    /**
     * Creates a command to stop the arm.
     *
     * @return A command that stops the arm
     */
    public Command stopCommand() {
        return runOnce(() -> setVelocity(0));
    }

    /**
     * Creates a command to move the arm at a specific velocity.
     *
     * @param velocityDegPerSec The target velocity in degrees per second
     * @return A command that moves the arm at the specified velocity
     */
    public Command moveAtVelocityCommand(double velocityDegPerSec) {
        return run(() -> setVelocity(velocityDegPerSec));
    }
}
