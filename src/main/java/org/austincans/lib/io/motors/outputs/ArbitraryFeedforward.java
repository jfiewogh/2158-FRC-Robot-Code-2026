package org.austincans.lib.io.motors.outputs;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** 
 * If motor requires different feedforward unit than what was set, feedforward will be zero.
 * For example, if the motor requires torque current and you call setVoltage instead of setTorqueCurrent, feedforward will be zero. 
 */
// TODO: Make a better way of handling this. It should be based on the PositionControlType or VelocityControlType.
public class ArbitraryFeedforward {
    public static enum FeedforwardType {
        NONE,
        DUTY_CYCLE,
        VOLTAGE,
        TORQUE_CURRENT,
    }

    private FeedforwardType feedforwardType;
    private double dutyCycle;
    private double volts;
    private double currentAmps;

    public void setDutyCycle(double dutyCycle) {
        feedforwardType = FeedforwardType.DUTY_CYCLE;
        this.dutyCycle = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        volts = 0.0;
        currentAmps = 0.0;
    }
    public void setVoltage(Voltage voltage) {
        feedforwardType = FeedforwardType.VOLTAGE;
        dutyCycle = 0.0;
        volts = voltage.in(Volts);
        currentAmps = 0.0;
    }
    public void setTorqueCurrent(Current current) {
        feedforwardType = FeedforwardType.TORQUE_CURRENT;
        dutyCycle = 0.0;
        volts = 0.0;
        currentAmps = current.in(Amps);
    }
    public void none() {
        feedforwardType = FeedforwardType.NONE;
        dutyCycle = 0.0;
        volts = 0.0;
        currentAmps = 0.0;
    }

    public FeedforwardType getFeedforwardType() {
        return feedforwardType;
    }
    public double getDutyCycle() {
        return dutyCycle;
    }
    public double getVolts() {
        return volts;
    }
    public double getCurrentAmps() {
        return currentAmps;
    }
}