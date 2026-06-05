package org.austincans.lib.io.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class BaseMotorIO<InputsType extends MotorIO.MotorIOInputs, ConfigType extends MotorConfig, ControlSlotType extends MotorIO.ControlSlot, VelocityControlTypeType extends MotorIO.VelocityControlType, PositionControlTypeType extends MotorIO.PositionControlType> implements MotorIO<InputsType, ConfigType, ControlSlotType, VelocityControlTypeType, PositionControlTypeType> {
    /** Updates inputs with newer values from motor */
    public void updateInputs(InputsType inputs) {
        inputs.setPosition(getPosition());
        inputs.setVelocity(getVelocity());
        inputs.setVoltage(getVoltage());
        inputs.setStatorCurrent(getStatorCurrent());
        inputs.setTemperature(getTemperature());
    }

    /** Applies output to motor based on the output mode */
    public final void applyOutputs(MotorIOOutputs<ControlSlotType, VelocityControlTypeType, PositionControlTypeType> outputs) {
        switch (outputs.getMode()) {
            case VOLTAGE -> setVoltage(outputs.getVoltage());
            case VELOCITY -> setVelocity(outputs.getVelocity(), outputs.getVelocityControlType(), outputs.getControlSlot(), outputs.getVoltage());
            case POSITION -> setPosition(outputs.getPosition(), outputs.getPositionControlType(), outputs.getControlSlot(), outputs.getVelocity(), outputs.getVoltage());
        }
    }

    /* Position */
    protected abstract Angle getPosition();
    protected abstract void setPosition(Angle position, PositionControlTypeType positionControlType, ControlSlotType slot, AngularVelocity velocity, Voltage arbitraryFeedforward);

    protected abstract AngularVelocity getVelocity();
    protected abstract void setVelocity(AngularVelocity velocity, VelocityControlTypeType velocityControlType, ControlSlotType slot, Voltage arbitraryFeedforward);

    protected abstract Voltage getVoltage();
    protected abstract void setVoltage(Voltage voltage);

    protected abstract Current getStatorCurrent();

    protected abstract Temperature getTemperature();
}
