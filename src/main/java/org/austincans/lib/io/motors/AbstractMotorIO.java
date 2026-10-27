package org.austincans.lib.io.motors;

import org.austincans.lib.io.Inputs;
import org.austincans.lib.io.Outputs;
import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.austincans.lib.io.motors.outputs.control.PositionControlInputs;
import org.austincans.lib.io.motors.outputs.control.VelocityControlInputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class AbstractMotorIO implements MotorIO {
    @Override
    public Inputs getInputs() {
        return new MotorIOInputs(
            getPosition(), getVelocity(), getVoltage(), getStatorCurrent(), getTemperature());
    }

    /** Applies output to motor based on the output mode */
    public final void applyOutputs(Outputs outputs) {
        MotorIOOutputs motorOutputs = (MotorIOOutputs) outputs;
        switch (motorOutputs.getMode()) {
            case VOLTAGE -> setVoltage(motorOutputs.getVoltage());
            case VELOCITY -> setVelocity(motorOutputs.getVelocityControlInputs());
            case POSITION -> setPosition(motorOutputs.getPositionControlInputs());
        }
    }

    /* Position */
    protected abstract Angle getPosition();
    protected abstract void setPosition(PositionControlInputs positionControlInputs);

    /* Velocity */
    protected abstract AngularVelocity getVelocity();
    protected abstract void setVelocity(VelocityControlInputs velocityControlInputs);

    /* Voltage */
    protected abstract Voltage getVoltage();
    protected abstract void setVoltage(Voltage voltage);

    protected abstract Current getStatorCurrent();

    protected abstract Temperature getTemperature();
}
