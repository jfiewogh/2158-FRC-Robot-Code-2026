package org.austincans.lib.io.motors.outputs.control;

import org.austincans.lib.io.motors.outputs.DutyCycleFeedforward;
import org.austincans.lib.io.motors.outputs.TorqueCurrentFeedforward;
import org.austincans.lib.io.motors.outputs.VoltageFeedforward;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TalonVelocityControlFeedforward extends VelocityControlFeedforward {
    public void dutyCycle(double ffOutput) {
        controlType = TalonVelocityControlType.DUTY_CYCLE;
        feedforward = new DutyCycleFeedforward(ffOutput);
    }
    public void voltage(Voltage ffVoltage) {
        controlType = TalonVelocityControlType.VOLTAGE;
        feedforward = new VoltageFeedforward(ffVoltage);
    }
    public void torqueCurrentFOC(Current ffCurrent) {
        controlType = TalonVelocityControlType.TORQUE_CURRENT_FOC;
        feedforward = new TorqueCurrentFeedforward(ffCurrent);
    }
    public void motionMagicDutyCycle(double ffOutput) {
        controlType = TalonVelocityControlType.MOTION_MAGIC_DUTY_CYCLE;
        feedforward = new DutyCycleFeedforward(ffOutput);
    }
    public void motionMagicVoltage(Voltage ffVoltage) {
        controlType = TalonVelocityControlType.MOTION_MAGIC_VOLTAGE;
        feedforward = new VoltageFeedforward(ffVoltage);
    }
    public void motionMagicTorqueCurrentFOC(Current ffCurrent) {
        controlType = TalonVelocityControlType.MOTION_MAGIC_TORQUE_CURRENT_FOC;
        feedforward = new TorqueCurrentFeedforward(ffCurrent);
    }
}
