package org.austincans.lib.io.motors.outputs.control;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class SparkVelocityControlInputs extends VelocityControlInputs {
    public void standardWithFFDutyCycle(AngularVelocity velocity, ControlSlot controlSlot, double ffOutput) {
        this.velocity = velocity;
        this.controlSlot = controlSlot;
        controlType = SparkVelocityControlType.STANDARD;
        arbitraryFeedforward.dutyCycle(ffOutput);
    }
    public void maxMotionWithFFDutyCycle(AngularVelocity velocity, ControlSlot controlSlot, double ffOutput) {
        this.velocity = velocity;
        this.controlSlot = controlSlot;
        controlType = SparkVelocityControlType.MAXMOTION;
        arbitraryFeedforward.dutyCycle(ffOutput);
    }
    public void standardWithFFVoltage(AngularVelocity velocity, ControlSlot controlSlot, Voltage ffVoltage) {
        this.velocity = velocity;
        this.controlSlot = controlSlot;
        controlType = SparkVelocityControlType.STANDARD;
        arbitraryFeedforward.voltage(ffVoltage);
    }
    public void maxMotionWithFFVoltage(AngularVelocity velocity, ControlSlot controlSlot, Voltage ffVoltage) {
        this.velocity = velocity;
        this.controlSlot = controlSlot;
        controlType = SparkVelocityControlType.MAXMOTION;
        arbitraryFeedforward.voltage(ffVoltage);
    }
}