package org.austincans.lib.io.motors.outputs.control;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.austincans.lib.io.motors.outputs.DutyCycleFeedforward;
import org.austincans.lib.io.motors.outputs.VoltageFeedforward;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class SparkPositionControlInputs extends PositionControlFeedforward {
    public void standardWithFFDutyCycle(AngularVelocity velocity, double kV, double ffOutput) {
        controlType = SparkPositionControlType.STANDARD;
        feedforward = new DutyCycleFeedforward(velocity.in(RPM) * kV + ffOutput);
    }
    public void standardWithFFDutyCycle(double ffOutput) {
        controlType = SparkPositionControlType.STANDARD;
        feedforward = new DutyCycleFeedforward(0.0);
    }

    public void maxMotionWithFFDutyCycle(AngularVelocity velocity, double kV, double ffOutput) {
        controlType = SparkPositionControlType.MAXMOTION;
        feedforward = new DutyCycleFeedforward(velocity.in(RPM) * kV + ffOutput);
    }
    public void maxMotionWithFFDutyCycle(double ffOutput) {
        controlType = SparkPositionControlType.MAXMOTION;
        feedforward = new DutyCycleFeedforward(0.0);
    }

    public void standardWithFFVoltage(AngularVelocity velocity, double kV, Voltage ffVoltage) {
        controlType = SparkPositionControlType.STANDARD;
        feedforward = new VoltageFeedforward(Volts.of(velocity.in(RPM) * kV).plus(ffVoltage));
    }
    public void standardWithFFVoltage(Voltage ffVoltage) {
        controlType = SparkPositionControlType.STANDARD;
        feedforward = new VoltageFeedforward(Volts.of(0.0));
    }
    
    public void maxMotionWithFFVoltage(AngularVelocity velocity, double kV, Voltage ffVoltage) {
        controlType = SparkPositionControlType.MAXMOTION;
        feedforward = new VoltageFeedforward(Volts.of(velocity.in(RPM) * kV).plus(ffVoltage));
    }
    public void maxMotionWithFFVoltage(Voltage ffVoltage) {
        controlType = SparkPositionControlType.MAXMOTION;
        feedforward = new VoltageFeedforward(Volts.of(0.0));
    }
}
