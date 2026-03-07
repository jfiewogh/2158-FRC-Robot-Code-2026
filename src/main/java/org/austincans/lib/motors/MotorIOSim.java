package org.austincans.lib.motors;

public class MotorIOSim implements MotorIO {
    private double appliedVolts;
    private double velocityRadPerSec;
    private double positionRad;

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRad = positionRad;
        inputs.velocityRadPerSec = velocityRadPerSec;
        inputs.appliedVolts = appliedVolts;
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVolts = voltage;
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        this.velocityRadPerSec = velocityRadPerSec;
    }

    @Override
    public void setPosition(double positionRad) {
        this.positionRad = positionRad;
    }
}
