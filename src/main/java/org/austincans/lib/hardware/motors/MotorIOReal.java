package org.austincans.lib.hardware.motors;

public abstract class MotorIOReal<InputsType extends MotorIO.MotorIOInputs> implements SelfCheckingMotorIO<InputsType> {
    final int deviceId;

    MotorIOReal(int deviceId) {
        this.deviceId = deviceId;
    }

    public int getDeviceId() {
        return deviceId;
    }
}
