package org.austincans.lib.io.motors.outputs;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** 
 * This clsas is immutable to reflect 2027 WPIlib's immutable-only measure implementations.
 * 
 * If motor requires different feedforward unit than what was set, feedforward will be zero.
 * For example, if the motor requires torque current and you call setVoltage instead of setTorqueCurrent, feedforward will be zero. 
 */
public class ArbitraryFeedforward {
    public static enum FeedforwardType {
        NONE,
        DUTY_CYCLE,
        VOLTAGE,
        TORQUE_CURRENT,
    }

    private static final Voltage zeroVoltage = Volts.of(0);
    private static final Current zeroCurrent = Amps.of(0);

    private FeedforwardType feedforwardType;
    private double dutyCycleOutput;
    private Voltage voltage;
    private Current current;

    public ArbitraryFeedforward() {
        none();
    }

    public void none() {
        feedforwardType = FeedforwardType.NONE;
        dutyCycleOutput = 0;
        voltage = zeroVoltage;
        current = zeroCurrent;
    }
    public void dutyCycle(double fractionalOutput) {
        feedforwardType = FeedforwardType.DUTY_CYCLE;
        dutyCycleOutput = fractionalOutput;
        voltage = zeroVoltage;
        current = zeroCurrent;
    }
    public void voltage(Voltage voltage) {
        feedforwardType = FeedforwardType.VOLTAGE;
        dutyCycleOutput = 0;
        this.voltage = voltage;
        current = zeroCurrent;
    }
    public void torqueCurrent(Current current) {
        feedforwardType = FeedforwardType.TORQUE_CURRENT;
        dutyCycleOutput = 0;
        voltage = zeroVoltage;
        this.current = current;
    }

    public FeedforwardType getFeedforwardType() {
        return feedforwardType;
    }
    public double getDutyCycleOutput() {
        return dutyCycleOutput;
    }
    public Voltage getVoltage() {
        return voltage;
    }
    public Current getCurrent() {
        return current;
    }
}