package org.austincans.lib.subsystems.drive.swerve;

import org.austincans.lib.io.motors.MotorIO;
import org.austincans.lib.io.sensors.encoders.AbsoluteEncoderIO;

public abstract class ModuleIOBase implements ModuleIO {
    private final MotorIO driveMotorIO;
    private final MotorIO steerMotorIO;
    private final AbsoluteEncoderIO steerAbsoluteEncoderIO;

    public ModuleIOBase(MotorIO driveMotorIO, MotorIO steerMotorIO, AbsoluteEncoderIO steerAbsoluteEncoderIO) {
        this.driveMotorIO = driveMotorIO;
        this.steerMotorIO = steerMotorIO;
        this.steerAbsoluteEncoderIO = steerAbsoluteEncoderIO;
    }

    @Override
    public void updateRecords(ModuleIOInputs inputs) {
        driveMotorIO.updateInputs(records.driveMotorRecords);
        steerMotorIO.updateInputs(records.steerMotorRecords);
        steerAbsoluteEncoderIO.updateInputs(records.steerAbsoluteEncoderRecords);
    }

    @Override
    public void applyRequests(ModuleIORequests requests) {
        driveMotorIO.applyRequests(requests.driveMotorRequests);
        steerMotorIO.applyRequests(requests.steerMotorRequests);
    }
}
