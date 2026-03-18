package org.austincans.lib.motors;

public class MotorIOFake implements MotorIO {
    private double appliedVolts;
    private double velocityRadPerSec;
    private double positionRad;

    @Override
    public void updateRecords(MotorIORecords records) {
        records.positionRad = positionRad;
        records.velocityRadPerSec = velocityRadPerSec;
        records.appliedVolts = appliedVolts;
    }

    @Override
    public void applyRequests(MotorIORequests requests) {
        appliedVolts = requests.voltage;
        velocityRadPerSec = requests.velocityRadPerSec;
        positionRad = requests.positionRad;
    }
}
