package org.austincans.lib.io.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class BaseMotorIO<
    InputsType extends MotorIO.MotorIOInputs, 
    ControlSlotType extends MotorIO.ControlSlot, 
    VelocityControlTypeType extends MotorIO.VelocityControlType, 
    PositionControlTypeType extends MotorIO.PositionControlType, 
    ConfigRequestsType extends MotorIO.MotorIOConfigRequests<ConfigType>,
    ConfigType
> implements MotorIO<
    InputsType, 
    ControlSlotType,
    VelocityControlTypeType, 
    PositionControlTypeType,
    ConfigRequestsType,
    ConfigType
> {
    /** Updates inputs with newer values from motor */
    public void updateInputs(InputsType inputs) {
        inputs.position = getPosition();
        inputs.velocity = getVelocity();
        inputs.voltage = getVoltage();
        inputs.statorCurrent = getStatorCurrent();
        inputs.temperature = getTemperature();
    }

    /** Applies output to motor based on the output mode */
    public final void applyOutputs(MotorIOOutputs<ControlSlotType, VelocityControlTypeType, PositionControlTypeType> outputs) {
        switch (outputs.mode) {
            case VOLTAGE -> setVoltage(outputs.voltage);
            case VELOCITY -> setVelocity(outputs.velocity, outputs.velocityControlType, outputs.controlSlot, outputs.arbitraryFeedforward);
            case POSITION -> setPosition(outputs.position, outputs.positionControlType, outputs.controlSlot, outputs.velocity, outputs.arbitraryFeedforward);
        }
    }

    /* Position */
    protected abstract Angle getPosition();
    protected abstract void setPosition(Angle position, PositionControlTypeType positionControlType, ControlSlotType slot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward);

    protected abstract AngularVelocity getVelocity();
    protected abstract void setVelocity(AngularVelocity velocity, VelocityControlTypeType velocityControlType, ControlSlotType slot, ArbitraryFeedforward arbitraryFeedforward);

    protected abstract Voltage getVoltage();
    protected abstract void setVoltage(Voltage voltage);

    protected abstract Current getStatorCurrent();

    protected abstract Temperature getTemperature();
}
