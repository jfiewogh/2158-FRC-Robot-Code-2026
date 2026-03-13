package org.austincans.lib.subsystems.drive;

import org.austincans.frc2026.subsystems.drive.ModuleIOInputsAutoLogged;
import org.austincans.lib.configs.ModuleConfig;
import org.austincans.lib.subsystems.drive.ModuleIO.ModuleIOInputs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Module extends AdvancedSubsystem {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final double wheelRadiusMeters;
    private double driveSpeedMultiplier;

    public Module(ModuleIO io, ModuleConfig moduleConfig) {
        this.io = io;
        wheelRadiusMeters = moduleConfig.wheelRadiusMeters();
        driveSpeedMultiplier = moduleConfig.driveSpeedMultiplier();
    }

    @Override // override to get rid of periodic 
    public void periodic() {
        io.updateInputs(inputs);
    }

    /** 
     * Runs the module with the specified setpoint state. 
     * Mutates the state to optimize it. 
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        Rotation2d steerAngle = getSteerAngle();
        state.optimize(steerAngle);
        state.cosineScale(steerAngle);

        // Apply setpoints
        double driveVelocityRadPerSec = state.speedMetersPerSecond / wheelRadiusMeters;
        io.setDriveVelocity(driveVelocityRadPerSec * driveSpeedMultiplier);
        io.setSteerAngle(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setSteerAngle(Rotation2d.kZero);
    }

    /** Runs the module with the specified output toward Angular Motions */
    public void runAngularCharacterization(double output, Translation2d moduleLocationMeters) {
        io.setDriveOpenLoop(output);
        io.setSteerAngle(
            new Rotation2d(moduleLocationMeters.getX(), moduleLocationMeters.getY())
                .plus(Rotation2d.kCCW_Pi_2));
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.stop();
    }

    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(inputs.steerAbsoluteEncoderInputs.absolutePositionRad);
    }

    /** Returns the current drive position of the module in meters. */
    private double getDrivePositionMeters() {
        return inputs.driveMotorInputs.positionRad * wheelRadiusMeters;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    private double getDriveVelocityMetersPerSec() {
        return inputs.driveMotorInputs.velocityRadPerSec * wheelRadiusMeters;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getSteerAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), getSteerAngle());
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.driveMotorInputs.positionRad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveMotorInputs.velocityRadPerSec);
    }

    public void setDriveSpeedMultiplier(double driveSpeedMultiplier) {
        this.driveSpeedMultiplier = driveSpeedMultiplier;
    }

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
