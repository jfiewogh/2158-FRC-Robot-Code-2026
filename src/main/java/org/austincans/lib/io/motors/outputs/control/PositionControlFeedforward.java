package org.austincans.lib.io.motors.outputs.control;

import org.austincans.lib.io.motors.outputs.Feedforward;

public abstract class PositionControlFeedforward {
    protected PositionControlType controlType;
    protected Feedforward feedforward;

    public PositionControlType getControlType() {
        return controlType;
    }
    public Feedforward getFeedforward() {
        return feedforward;
    }
}