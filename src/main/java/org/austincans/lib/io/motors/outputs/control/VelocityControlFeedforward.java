package org.austincans.lib.io.motors.outputs.control;

import org.austincans.lib.io.motors.outputs.Feedforward;

public abstract class VelocityControlFeedforward {
    protected VelocityControlType controlType;
    protected Feedforward feedforward;

    public VelocityControlType getControlType() {
        return controlType;
    }
    public Feedforward getFeedforward() {
        return feedforward;
    }
}