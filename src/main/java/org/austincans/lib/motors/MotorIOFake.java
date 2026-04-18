package org.austincans.lib.motors;

public class MotorIOFake implements MotorIO {
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
    public void applyOutputs(MotorIOOutputs outputs) {
        appliedVolts = outputs.voltage;
        velocityRadPerSec = outputs.velocityRadPerSec;
        positionRad = outputs.positionRad;
    }
}
