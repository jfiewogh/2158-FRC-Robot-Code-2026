package org.austincans.lib.motors;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class MotorIOFake implements MotorIO {
    private Voltage appliedVolts;
    private AngularVelocity velocity;
    private Angle position;

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.position = position;
        inputs.velocity = velocity;
        inputs.voltage = appliedVolts;
    }

    @Override
    public void applyOutputs(MotorIOOutputs outputs) {
        appliedVolts = outputs.voltage;
        velocity = outputs.velocity;
        position = outputs.position;
    }
}
