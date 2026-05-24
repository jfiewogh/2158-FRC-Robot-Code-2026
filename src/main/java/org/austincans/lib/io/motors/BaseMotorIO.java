package org.austincans.lib.io.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public abstract class BaseMotorIO<InputsType extends MotorIO.MotorIOInputs> implements MotorIO<InputsType> {
    public final void applyOutputs(MotorIOOutputs outputs) {
        switch (outputs.getMode()) {
            case VOLTAGE -> setVoltage(outputs.getVoltage());
            case VELOCITY -> setVelocity(outputs.getVelocity());
            case POSITION -> setPosition(outputs.getPosition());
        }
    }

    abstract void setVoltage(Voltage voltage);

    abstract void setVelocity(AngularVelocity velocity);

    abstract void setPosition(Angle position);
}
