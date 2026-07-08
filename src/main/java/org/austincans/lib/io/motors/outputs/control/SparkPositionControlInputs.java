package org.austincans.lib.io.motors.outputs.control;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class SparkPositionControlInputs extends PositionControlInputs {
    public void standardWithFFDutyCycle(Angle position, ControlSlot controlSlot, AngularVelocity velocity, double kV, double ffOutput) {
        this.position = position;
        this.controlSlot = controlSlot;
        controlType = SparkPositionControlType.STANDARD;
        arbitraryFeedforward.dutyCycle(velocity.in(RPM) * kV + ffOutput);
    }
    public void standardWithFFDutyCycle(Angle position, ControlSlot controlSlot, double ffOutput) {
        this.position = position;
        this.controlSlot = controlSlot;
        controlType = SparkPositionControlType.STANDARD;
        arbitraryFeedforward.none();
    }

    public void maxMotionWithFFDutyCycle(Angle position, ControlSlot controlSlot, AngularVelocity velocity, double kV, double ffOutput) {
        this.position = position;
        this.controlSlot = controlSlot;
        controlType = SparkPositionControlType.MAXMOTION;
        arbitraryFeedforward.dutyCycle(velocity.in(RPM) * kV + ffOutput);
    }
    public void maxMotionWithFFDutyCycle(Angle position, ControlSlot controlSlot, double ffOutput) {
        this.position = position;
        this.controlSlot = controlSlot;
        controlType = SparkPositionControlType.MAXMOTION;
        arbitraryFeedforward.none();
    }

    public void standardWithFFVoltage(Angle position, ControlSlot controlSlot, AngularVelocity velocity, double kV, Voltage ffVoltage) {
        this.position = position;
        this.controlSlot = controlSlot;
        controlType = SparkPositionControlType.STANDARD;
        arbitraryFeedforward.voltage(Volts.of(velocity.in(RPM) * kV).plus(ffVoltage));
    }
    public void standardWithFFVoltage(Angle position, ControlSlot controlSlot, Voltage ffVoltage) {
        this.position = position;
        this.controlSlot = controlSlot;
        controlType = SparkPositionControlType.STANDARD;
        arbitraryFeedforward.none();
    }
    
    public void maxMotionWithFFVoltage(Angle position, ControlSlot controlSlot, AngularVelocity velocity, double kV, Voltage ffVoltage) {
        this.position = position;
        this.controlSlot = controlSlot;
        controlType = SparkPositionControlType.MAXMOTION;
        arbitraryFeedforward.voltage(Volts.of(velocity.in(RPM) * kV).plus(ffVoltage));
    }
    public void maxMotionWithFFVoltage(Angle position, ControlSlot controlSlot, Voltage ffVoltage) {
        this.position = position;
        this.controlSlot = controlSlot;
        controlType = SparkPositionControlType.MAXMOTION;
        arbitraryFeedforward.none();
    }
}
