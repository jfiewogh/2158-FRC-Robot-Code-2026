package org.austincans.lib.subsystems.drive;

import org.austincans.lib.configs.ModuleConfig;
import org.austincans.lib.encoders.AbsoluteEncoderIO;
import org.austincans.lib.encoders.EncoderIO;
import org.austincans.lib.motors.MotorIO;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class ModuleIOBase<T extends MotorIO, U extends MotorIO, V extends AbsoluteEncoderIO> implements ModuleIO {
    private final T driveMotorIO;
    private final U steerMotorIO;
    private final V steerAbsoluteEncoderIO;

    public ModuleIOBase(ModuleConfig moduleConfig, T driveMotorIO, U steerMotorIO, V steerAbsoluteEncoderIO) {
        this.driveMotorIO = driveMotorIO;
        this.steerMotorIO = steerMotorIO;
        this.steerAbsoluteEncoderIO = steerAbsoluteEncoderIO;
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveMotorIO.updateInputs(inputs.driveMotorInputs);
        steerMotorIO.updateInputs(inputs.steerMotorInputs);
        steerAbsoluteEncoderIO.updateInputs(inputs.steerEncoderInputs);
    }

    @Override
    public void runSetpoint(SwerveModuleState state) {
        state.optimize(getAngle());
        driveMotor.setVelocity();
        steerMotor.setPosition(getPositionMeters());
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
        return driveMotor.ModuleIOInputs.positionRad * moduleConfig.WheelRadius;
    }
}
