package org.austincans.lib.io.motors;

import org.austincans.lib.io.Inputs;
import org.austincans.lib.io.Outputs;
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

public abstract class AbstractMotorIO implements MotorIO {
    /** Updates inputs with newer values from motor */
    @Override
    public void updateInputs(Inputs inputs) {
        MotorIOInputs motorInputs = (MotorIOInputs) inputs;
        motorInputs.position = getPosition();
        motorInputs.velocity = getVelocity();
        motorInputs.voltage = getVoltage();
        motorInputs.statorCurrent = getStatorCurrent();
        motorInputs.temperature = getTemperature();
    }

    /** Applies output to motor based on the output mode */
    public final void applyOutputs(Outputs outputs) {
        MotorIOOutputs motorOutputs = (MotorIOOutputs) outputs;
        switch (motorOutputs.getMode()) {
            case VOLTAGE -> setVoltage(motorOutputs.getVoltage());
            case VELOCITY -> setVelocity(
                motorOutputs.getVelocity(), motorOutputs.getVelocityControlType(), motorOutputs.getControlSlot(), 
                motorOutputs.getArbitraryFeedforward());
            case POSITION -> setPosition(
                motorOutputs.getPosition(), motorOutputs.getPositionControlType(), motorOutputs.getControlSlot(), 
                motorOutputs.getVelocity(), motorOutputs.getArbitraryFeedforward());
        }
    }

    /* Position */
    protected abstract Angle getPosition();
    protected abstract void setPosition(Angle position, PositionControlType positionControlType, ControlSlot slot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward);

    /* Velocity */
    protected abstract AngularVelocity getVelocity();
    protected abstract void setVelocity(AngularVelocity velocity, VelocityControlType velocityControlType, ControlSlot slot, ArbitraryFeedforward arbitraryFeedforward);

    /* Voltage */
    protected abstract Voltage getVoltage();
    protected abstract void setVoltage(Voltage voltage);

    protected abstract Current getStatorCurrent();

    protected abstract Temperature getTemperature();
}
