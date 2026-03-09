package org.austincans.lib.subsystems.drive;

import org.austincans.lib.encoders.AbsoluteEncoderIO;
import org.austincans.lib.motors.MotorIO;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class ModuleIOBase<T extends MotorIO, U extends MotorIO, V extends AbsoluteEncoderIO> implements ModuleIO {
    private final T driveMotorIO;
    private final U steerMotorIO;
    private final V steerAbsoluteEncoderIO;

    public ModuleIOBase(T driveMotorIO, U steerMotorIO, V steerAbsoluteEncoderIO) {
        this.driveMotorIO = driveMotorIO;
        this.steerMotorIO = steerMotorIO;
        this.steerAbsoluteEncoderIO = steerAbsoluteEncoderIO;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveMotorIO.updateInputs(inputs.driveMotorInputs);
        steerMotorIO.updateInputs(inputs.steerMotorInputs);
        steerAbsoluteEncoderIO.updateInputs(inputs.steerAbsoluteEncoderInputs);
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotorIO.setVoltage(output);
    }

    @Override
    public void setSteerOpenLoop(double output) {
        steerMotorIO.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveMotorIO.setVelocity(velocityRadPerSec);
    }

    @Override
    public void setSteerAngle(Rotation2d angle) {
        steerMotorIO.setPosition(angle.getRadians());
    }

    @Override
    public void stop() {
        driveMotorIO.setVoltage(0);
        steerMotorIO.setVoltage(0);
    }
}
