package org.austincans.lib.hardware.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * This class is currently unused. It will probably be replaced or modified when sim is implemented.
 */
public class MotorIOFake implements MotorIO<MotorIO.MotorIOInputs> {
    private Angle position;
    private AngularVelocity velocity;
    private Voltage appliedVolts;

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.setPosition(position);
        inputs.setVelocity(velocity);
        inputs.setVoltage(appliedVolts);
    }

    @Override
    public void applyOutputs(MotorIOOutputs outputs) {
        appliedVolts = outputs.getVoltage();
        velocity = outputs.getVelocity();
        position = outputs.getPosition();
    }
}
