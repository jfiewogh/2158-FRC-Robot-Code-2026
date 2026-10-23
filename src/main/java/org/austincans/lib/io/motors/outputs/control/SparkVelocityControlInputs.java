package org.austincans.lib.io.motors.outputs.control;

import org.austincans.lib.io.motors.outputs.DutyCycleFeedforward;
import org.austincans.lib.io.motors.outputs.VoltageFeedforward;

import edu.wpi.first.units.measure.Voltage;

public class SparkVelocityControlInputs extends VelocityControlFeedforward {
    public void standardWithFFDutyCycle(double ffOutput) {
        controlType = SparkVelocityControlType.STANDARD;
        feedforward = new DutyCycleFeedforward(ffOutput);
    }
    public void maxMotionWithFFDutyCycle(double ffOutput) {
        controlType = SparkVelocityControlType.MAXMOTION;
        feedforward = new DutyCycleFeedforward(ffOutput);
    }
    public void standardWithFFVoltage(Voltage ffVoltage) {
        controlType = SparkVelocityControlType.STANDARD;
        feedforward = new VoltageFeedforward(ffVoltage);
    }
    public void maxMotionWithFFVoltage(Voltage ffVoltage) {
        controlType = SparkVelocityControlType.MAXMOTION;
        feedforward = new VoltageFeedforward(ffVoltage);
    }
}