package org.austincans.lib.io.sensors.gyros;

import org.austincans.lib.io.Inputs;
import org.austincans.lib.io.sensors.gyros.inputs.GyroIOInputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public abstract class AbstractGyroIO implements GyroIO {
    @Override
    public void updateInputs(Inputs inputs) {
        GyroIOInputs gyroInputs = (GyroIOInputs) inputs;
        gyroInputs.connected = isConnected();
        gyroInputs.yawPosition = getYawPosition();
        gyroInputs.pitchPosition = getPitchPosition();
        gyroInputs.rollPosition = getRollPosition();
        gyroInputs.yawVelocity = getYawVelocity();
        gyroInputs.pitchVelocity = getPitchVelocity();
        gyroInputs.rollVelocity = getRollVelocity();
    }

    protected abstract boolean isConnected();

    protected abstract Angle getYawPosition();
    protected abstract Angle getPitchPosition();
    protected abstract Angle getRollPosition();

    protected abstract AngularVelocity getYawVelocity();
    protected abstract AngularVelocity getPitchVelocity();
    protected abstract AngularVelocity getRollVelocity();
}
