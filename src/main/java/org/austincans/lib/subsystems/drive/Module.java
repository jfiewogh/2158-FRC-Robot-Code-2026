package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.ModuleConfig;
import org.austincans.lib.subsystems.drive.ModuleIO.ModuleIOInputs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputs inputs;
    private final double wheelRadiusMeters;
    private double driveSpeedMultiplier;

    public Module(ModuleIO io, ModuleIOInputs inputs, ModuleConfig moduleConfig) {
        this.io = io;
        this.inputs = inputs;
        wheelRadiusMeters = moduleConfig.wheelRadiusMeters();
        driveSpeedMultiplier = moduleConfig.driveSpeedMultiplier();
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(inputs.steerAbsoluteEncoderInputs.absolutePositionRad);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
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

    public void setDriveSpeedMultiplier(double driveSpeedMultiplier) {
        this.driveSpeedMultiplier = driveSpeedMultiplier;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }
}
