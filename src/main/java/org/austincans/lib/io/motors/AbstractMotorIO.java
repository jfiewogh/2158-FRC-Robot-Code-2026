package org.austincans.lib.io.motors;

import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.ArbitraryFeedforward;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.austincans.lib.io.motors.outputs.control.ControlSlot;
import org.austincans.lib.io.motors.outputs.control.PositionControlType;
import org.austincans.lib.io.motors.outputs.control.VelocityControlType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class AbstractMotorIO<
    InputsType extends MotorIOInputs, 
    ControlSlotType extends ControlSlot, 
    VelocityControlTypeType extends VelocityControlType, 
    PositionControlTypeType extends PositionControlType, 
    ConfigRequestsType extends ConfigRequests<ConfigType>,
    ConfigType> 
implements MotorIO<
    InputsType, 
    ControlSlotType,
    VelocityControlTypeType, 
    PositionControlTypeType,
    ConfigRequestsType,
    ConfigType> 
{
    /** Updates inputs with newer values from motor */
    @Override
    public void updateInputs(InputsType inputs) {
        inputs.position = getPosition();
        inputs.velocity = getVelocity();
        inputs.voltage = getVoltage();
        inputs.statorCurrent = getStatorCurrent();
        inputs.temperature = getTemperature();
    }

    /** Applies output to motor based on the output mode */
    public final void applyOutputs(MotorIOOutputs<ControlSlotType, VelocityControlTypeType, PositionControlTypeType> outputs) {
        switch (outputs.getMode()) {
            case VOLTAGE -> setVoltage(outputs.getVoltage());
            case VELOCITY -> setVelocity(
                outputs.getVelocity(), outputs.getVelocityControlType(), outputs.getControlSlot(), 
                outputs.getArbitraryFeedforward());
            case POSITION -> setPosition(
                outputs.getPosition(), outputs.getPositionControlType(), outputs.getControlSlot(), 
                outputs.getVelocity(), outputs.getArbitraryFeedforward());
        }
    }

    /* Position */
    protected abstract Angle getPosition();
    protected abstract void setPosition(Angle position, PositionControlTypeType positionControlType, ControlSlotType slot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward);

    /* Velocity */
    protected abstract AngularVelocity getVelocity();
    protected abstract void setVelocity(AngularVelocity velocity, VelocityControlTypeType velocityControlType, ControlSlotType slot, ArbitraryFeedforward arbitraryFeedforward);

    /* Voltage */
    protected abstract Voltage getVoltage();
    protected abstract void setVoltage(Voltage voltage);

    protected abstract Current getStatorCurrent();

    protected abstract Temperature getTemperature();
}
