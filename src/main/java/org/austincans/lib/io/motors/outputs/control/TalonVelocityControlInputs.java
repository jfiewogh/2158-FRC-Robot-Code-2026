package org.austincans.lib.io.motors.outputs.control;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TalonVelocityControlInputs extends VelocityControlInputs {
    public void dutyCycle(AngularVelocity velocity, ControlSlot controlSlot, double fractionalOutput) {
        this.velocity = velocity;
        this.controlSlot = controlSlot;
        controlType = TalonVelocityControlType.DUTY_CYCLE;
        arbitraryFeedforward.dutyCycle(fractionalOutput);
    }
    public void voltage(AngularVelocity velocity, ControlSlot controlSlot, Voltage voltage) {
        this.velocity = velocity;
        this.controlSlot = controlSlot;
        controlType = TalonVelocityControlType.VOLTAGE;
        arbitraryFeedforward.voltage(voltage);
    }
    public void torqueCurrentFOC(AngularVelocity velocity, ControlSlot controlSlot, Current current) {
        this.velocity = velocity;
        this.controlSlot = controlSlot;
        controlType = TalonVelocityControlType.TORQUE_CURRENT_FOC;
        arbitraryFeedforward.torqueCurrent(current);
    }
    public void motionMagicDutyCycle(AngularVelocity velocity, ControlSlot controlSlot, double fractionalOutput) {
        this.velocity = velocity;
        this.controlSlot = controlSlot;
        controlType = TalonVelocityControlType.MOTION_MAGIC_DUTY_CYCLE;
        arbitraryFeedforward.dutyCycle(fractionalOutput);
    }
    public void motionMagicVoltage(AngularVelocity velocity, ControlSlot controlSlot, Voltage voltage) {
        this.velocity = velocity;
        this.controlSlot = controlSlot;
        controlType = TalonVelocityControlType.MOTION_MAGIC_VOLTAGE;
        arbitraryFeedforward.voltage(voltage);
    }
    public void motionMagicTorqueCurrentFOC(AngularVelocity velocity, ControlSlot controlSlot, Current current) {
        this.velocity = velocity;
        this.controlSlot = controlSlot;
        controlType = TalonVelocityControlType.MOTION_MAGIC_TORQUE_CURRENT_FOC;
        arbitraryFeedforward.torqueCurrent(current);
    }
}
