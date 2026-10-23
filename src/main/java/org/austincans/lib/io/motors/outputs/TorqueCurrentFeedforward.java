package org.austincans.lib.io.motors.outputs;

import edu.wpi.first.units.measure.Current;

public class TorqueCurrentFeedforward implements Feedforward {
    private final Current current;

    public TorqueCurrentFeedforward(Current current) {
        this.current = current;
    }

    public Current get() {
        return current;
    }
}
