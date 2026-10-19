package org.austincans.lib.io.motors.outputs;

import edu.wpi.first.math.MathUtil;

public class DutyCycleFeedforward implements Feedforward {
    private final double output;

    public DutyCycleFeedforward(double output) {
        this.output = MathUtil.clamp(output, 0.0, 1.0);
    }

    public double get() {
        return output;
    }
}
