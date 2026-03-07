package org.austincans.lib.subsystems.drive;

import org.austincans.lib.encoders.EncoderIO;
import org.austincans.lib.motors.MotorIO;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class ModuleIOBase<DriveMotor extends MotorIO, SteerMotor extends MotorIO, Encoder extends EncoderIO> implements ModuleIO {
    private final DriveMotor driveMotor;
    private final SteerMotor steerMotor;
    private final Encoder steerEncoder;

    public ModuleIOBase(ModuleConfig moduleConfig, DriveMotor driveMotor, SteerMotor steerMotor, Encoder steerEncoder) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveMotor.updateInputs(inputs.driveMotorInputs);
        steerMotor.updateInputs(inputs.steerMotorInputs);
        steerEncoder.updateInputs(inputs.steerEncoderInputs);
        inputs.

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
        return driveMotor.
    }
}
