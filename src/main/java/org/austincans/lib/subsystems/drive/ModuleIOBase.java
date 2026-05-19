package org.austincans.lib.subsystems.drive;

import org.austincans.lib.motors.MotorIO;
import org.austincans.lib.sensors.encoders.AbsoluteEncoderIO;

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
    public void updateRecords(ModuleIORecords records) {
        driveMotorIO.updateRecords(records.driveMotorRecords);
        steerMotorIO.updateRecords(records.steerMotorRecords);
        steerAbsoluteEncoderIO.updateInputs(records.steerAbsoluteEncoderRecords);
    }

    @Override
    public void applyRequests(ModuleIORequests requests) {
        driveMotorIO.applyRequests(requests.driveMotorRequests);
        steerMotorIO.applyRequests(requests.steerMotorRequests);
    }
}
