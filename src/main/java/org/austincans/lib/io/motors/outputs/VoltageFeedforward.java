package org.austincans.lib.io.motors.outputs;

import edu.wpi.first.units.measure.Voltage;

public class VoltageFeedforward implements Feedforward {
    private final Voltage voltage;

    public VoltageFeedforward(Voltage voltage) {
        this.voltage = voltage;
    }

    public Voltage get() {
        return voltage;
    }
}
