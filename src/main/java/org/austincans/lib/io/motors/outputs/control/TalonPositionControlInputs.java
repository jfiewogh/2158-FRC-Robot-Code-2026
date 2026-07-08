package org.austincans.lib.io.motors.outputs.control;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TalonPositionControlInputs extends PositionControlInputs {
    public void dutyCycle(Angle position, ControlSlot controlSlot, AngularVelocity velocity, double ffOutput) {
        controlType = TalonPositionControlType.DUTY_CYCLE;
        this.position = position;
        this.controlSlot = controlSlot;
        this.velocity = velocity;
        arbitraryFeedforward.dutyCycle(ffOutput);
    }
    public void voltage(Angle position, ControlSlot controlSlot, AngularVelocity velocity, Voltage ffVoltage) {
        controlType = TalonPositionControlType.VOLTAGE;
        this.position = position;
        this.controlSlot = controlSlot;
        this.velocity = velocity;
        arbitraryFeedforward.voltage(ffVoltage);
    }
    public void torqueCurrentFOC(Angle position, AngularVelocity velocity, ControlSlot controlSlot, Current ffTorqueCurrent) {
        controlType = TalonPositionControlType.TORQUE_CURRENT_FOC;
        this.position = position;
        this.controlSlot = controlSlot;
        this.velocity = velocity;
        arbitraryFeedforward.torqueCurrent(ffTorqueCurrent);
    }

    public void motionMagicDutyCycle(Angle position, ControlSlot controlSlot, double ffOutput) {
        controlType = TalonPositionControlType.MOTION_MAGIC_DUTY_CYCLE;
        this.position = position;
        this.controlSlot = controlSlot;
        arbitraryFeedforward.dutyCycle(ffOutput);
    }
    public void motionMagicVoltage(Angle position, ControlSlot controlSlot, Voltage ffVoltage) {
        controlType = TalonPositionControlType.MOTION_MAGIC_VOLTAGE;
        this.position = position;
        this.controlSlot = controlSlot;
        arbitraryFeedforward.voltage(ffVoltage);
    }
    public void motionMagicTorqueCurrentFOC(Angle position, ControlSlot controlSlot, Current ffTorqueCurrent) {
        controlType = TalonPositionControlType.MOTION_MAGIC_TORQUE_CURRENT_FOC;
        this.position = position;
        this.controlSlot = controlSlot;
        arbitraryFeedforward.torqueCurrent(ffTorqueCurrent);
    }

    public void motionMagicExpoDutyCycle(Angle position, ControlSlot controlSlot, double ffOutput) {
        controlType = TalonPositionControlType.MOTION_MAGIC_EXPO_DUTY_CYCLE;
        this.position = position;
        this.controlSlot = controlSlot;
        arbitraryFeedforward.dutyCycle(ffOutput);
    }
    public void motionMagicExpoVoltage(Angle position, ControlSlot controlSlot, Voltage ffVoltage) {
        controlType = TalonPositionControlType.MOTION_MAGIC_EXPO_VOLTAGE;
        this.position = position;
        this.controlSlot = controlSlot;
        arbitraryFeedforward.voltage(ffVoltage);
    }
    public void motionMagicExpoTorqueCurrentFOC(Angle position, ControlSlot controlSlot, Current ffTorqueCurrent) {
        controlType = TalonPositionControlType.MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC;
        this.position = position;
        this.controlSlot = controlSlot;
        arbitraryFeedforward.torqueCurrent(ffTorqueCurrent);
    }
}
