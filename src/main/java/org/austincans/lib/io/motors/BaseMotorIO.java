package org.austincans.lib.io.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class BaseMotorIO<InputsType extends MotorIO.MotorIOInputs, ControlSlotType extends MotorIO.ControlSlot> implements MotorIO<InputsType, ControlSlotType> {
    public void updateInputs(InputsType inputs) {
        inputs.setPosition(getPosition());
        inputs.setVelocity(getVelocity());
        inputs.setVoltage(getVoltage());
        inputs.setStatorCurrent(getStatorCurrent());
        inputs.setTemperature(getTemperature());
    }

    public final void applyOutputs(MotorIOOutputs outputs) {
        switch (outputs.getMode()) {
            case VOLTAGE -> setVoltage(outputs.getVoltage());
            case VELOCITY -> setVelocity(outputs.getVelocity());
            case POSITION -> setPosition(outputs.getPosition());
        }
    }

    abstract Angle getPosition();
    abstract AngularVelocity getVelocity();
    abstract Voltage getVoltage();
    abstract Current getStatorCurrent();
    abstract Temperature getTemperature();

    abstract void setVoltage(Voltage voltage);
    abstract void setVelocity(AngularVelocity velocity);
    abstract void setPosition(Angle position);
}
