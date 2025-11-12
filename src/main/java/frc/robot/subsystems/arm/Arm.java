package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs;

    /* Constants (TODO: REPLACE ME WITH YAML) */
    /*
     * Feedforward
     * kS: Static
     * kG: Gravity
     * kV: Velocity
     * kA: Acceleration
     */
    private double kS = 0;
    private double kG = 0;
    private double kV = 0;
    private double kA = 0;
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

    /* Constants (TODO: REPLACE ME WITH YAML) */
    /*
     * PID
     * kP: Proportional
     * kI: Integral
     * kD: Derivative
     */
    private final double kP = 1;
    private final double kI = 0;
    private final double kD = 0;
    private TrapezoidProfile.Constraints constraints;
    private ProfiledPIDController armPIDController = new ProfiledPIDController(kP, kI, kD, constraints);
    private double targetPosition = 0.0;
    private double targetVelocity = 0.0;

    private double maxVelocity = 0;
    private double maxAcceleration = 0;
    private double gearRatio = 0;

    public Arm(ArmIO io) {
        this.io = io;

        // Initialize ProfiledPIDController
        // Convert from radians to rotations for constraints
        double maxVelocityRotations = maxVelocity / (2.0 * Math.PI);
        double maxAccelerationRotations = maxAcceleration / (2.0 * Math.PI);

        constraints = new TrapezoidProfile.Constraints(maxVelocityRotations, maxAccelerationRotations);
        armPIDController = new ProfiledPIDController(kP, kI, kD, constraints);
    }

    public void calculate() {
        double currentPos = getPosition();
        double output = armPIDController.calculate(currentPos, targetPosition);
        double velocity = armPIDController.getSetpoint().velocity;
        double angle = getPositionRadians();
        double feedforwardOutput = feedforward.calculate(angle, velocity);
        io.setVoltage(output + feedforwardOutput);
    }

    public double getPosition() {
        return inputs.position / gearRatio;
    }

    public double getVelocity() {
        return inputs.position / gearRatio / 60.0; // Convert from RPM to RPS
    }

    public void setVelocity(double velocityDegPerSec) {
        setVelocity(velocityDegPerSec, 0);
    }

    public void setVelocity(double velocityDegPerSec, double acceleration) {
        // Convert degrees/sec to rotations/sec
        double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
        double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

        // Use the ProfiledPIDController
        targetVelocity = velocityRotations;

        // If acceleration is specified, update constraints
        if (acceleration > 0) {
            double maxAccelRotations = Units.degreesToRadians(acceleration) / (2.0 * Math.PI);
            constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAccelRotations);
            armPIDController.setConstraints(constraints);
        }

        // Apply velocity directly to the motor controller as well
        // This ensures immediate response while the control loop refines it
        double armAngle = getPositionRadians();
        double ffVolts = feedforward.calculateWithVelocities(armAngle, velocityRotations, 0);
        io.setVoltage(ffVolts);
    }

    public Command moveToAngleCommand(double angleDegrees) {
        return new ConditionalCommand(
                        new InstantCommand(() -> {
                            setAngle(angleDegrees);
                        }),
                        null,
                        () -> {
                            double currentAngle = Units.radiansToDegrees(getPositionRadians());
                            return Math.abs(angleDegrees - currentAngle) < 2.0;
                        })
                .finallyDo(interrupted -> setVelocity(0));
    }

    public Command stopCommand() {
        return new InstantCommand(() -> setVelocity(0));
    }

    public Command moveAtVelocityCommand(double velocityDegPerSec) {
        return new InstantCommand(() -> setVelocity(velocityDegPerSec));
    }

    public double getPositionRadians() {
        double rotations = getPosition();
        return rotations * 2.0 * Math.PI;
    }

    public double getAmps() {
        return inputs.currentAmps;
    }

    public double getCurrentVoltage() {
        return inputs.currentVoltage;
    }

    public double getTemperature() {
        return inputs.temperature;
    }

    public void setAngle(double angleDegrees) {
        setAngle(angleDegrees, 0);
    }

    public void setAngle(double angleDegrees, double acceleration) {
        // Convert degrees to rotations
        double angleRadians = Units.degreesToRadians(angleDegrees);
        double positionRotations = angleRadians / (2.0 * Math.PI);

        // Use the ProfiledPIDController
        targetPosition = positionRotations;

        // If acceleration is specified, update constraints
        if (acceleration > 0) {
            double maxAccelRotations = acceleration / (2.0 * Math.PI);
            constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAccelRotations);
            armPIDController.setConstraints(constraints);
        }
    }

    public Command setAngleCommand(double angleDegrees) {
        return new InstantCommand(() -> setAngle(angleDegrees));
    }

    // Feedforward
    public double calculate(double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {

        return feedforward.getKs() * Math.signum(velocityRadPerSec)
                + feedforward.getKg() * Math.cos(positionRadians)
                + feedforward.getKv() * velocityRadPerSec
                + feedforward.getKa() * accelRadPerSecSquared;
    }

    private String controlMode = "Position";

    public void controlLoopFn() {
        switch (controlMode) {
            case "Position":
                double currentPos = getPosition();
                double output = armPIDController.calculate(currentPos, targetPosition);
                double velocity = armPIDController.getSetpoint().velocity;
                double angle = getPositionRadians();
                double feedforwardOutput = feedforward.calculate(angle, velocity);
                io.setVoltage(output + feedforwardOutput);
                break;
            case "Velocity":
                double currentVel = getVelocity();
                double velOutput = armPIDController.calculate(currentVel, targetVelocity);
                double accel = armPIDController.getSetpoint().velocity - currentVel;
                double armAngle = getPositionRadians();
                double velFeedforwardOutput = calculate(armAngle, targetVelocity, accel);

                // Apply the combined PID output and feedforward to the motor
                double velocityVoltage = velOutput + velFeedforwardOutput;
                io.setVoltage(velocityVoltage);
                break;
            default:
                // Do nothing, voltage is set directly
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
