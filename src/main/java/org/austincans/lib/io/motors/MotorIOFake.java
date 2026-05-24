package org.austincans.lib.io.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * This class is currently unused. It will probably be replaced or modified when sim is implemented.
 */
public class MotorIOFake extends BaseMotorIO<MotorIO.MotorIOInputs> {
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
    void setVoltage(Voltage voltage) {
        appliedVolts = voltage;
    }

    @Override
    void setVelocity(AngularVelocity velocity) {
        this.velocity = velocity;
    }

    @Override
    void setPosition(Angle position) {
        this.position = position;
    }
}
