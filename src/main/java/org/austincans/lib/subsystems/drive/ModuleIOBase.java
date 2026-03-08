package org.austincans.lib.subsystems.drive;

import org.austincans.frc2026.subsystems.drive.ModuleIOInputsAutoLogged;
import org.austincans.lib.configs.ModuleConfig;
import org.austincans.lib.encoders.AbsoluteEncoderIO;
import org.austincans.lib.motors.MotorIO;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class ModuleIOBase<T extends MotorIO, U extends MotorIO, V extends AbsoluteEncoderIO> implements ModuleIO {
    private final T driveMotorIO;
    private final ModuleIOInputsAutoLogged driveMotorInputs = new ModuleIOInputsAutoLogged();
    private final U steerMotorIO;
    private final V steerAbsoluteEncoderIO;
    private final double wheelRadiusMeters;

    private final double driveSpeedMultiplier;

    public ModuleIOBase(ModuleConfig moduleConfig, T driveMotorIO, U steerMotorIO, V steerAbsoluteEncoderIO) {
        this.driveMotorIO = driveMotorIO;
        this.steerMotorIO = steerMotorIO;
        this.steerAbsoluteEncoderIO = steerAbsoluteEncoderIO;
        wheelRadiusMeters = moduleConfig.wheelRadiusMeters();
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveMotorInputs = driveMotorInputs;
        driveMotorIO.updateInputs(inputs.driveMotorInputs);
        steerMotorIO.updateInputs(inputs.steerMotorInputs);
        steerAbsoluteEncoderIO.updateInputs(inputs.steerEncoderInputs);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    @Override
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        Rotation2d steerAngle = getSteerAngle();
        state.optimize(steerAngle);
        state.cosineScale(steerAngle);

        // Apply setpoints
        double driveVelocityRadPerSec = state.speedMetersPerSecond / wheelRadiusMeters;
        driveMotorIO.setVelocity(driveVelocityRadPerSec * driveSpeedMultiplier);
        steerMotorIO.setPosition(state.angle.getRadians());
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    private double getPositionMeters() {
        return driveMotorIO. * wheelRadius;
    }

    private double getVelocityMetersPerSec() {
        return driveMotorIO * wheelRadius;
    } 

    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steerAbsoluteEncoderIO.getAbsolutePositionRotations());
    }

    public void setDriveSpeedCoefficient() {

    }
}
