package org.austincans.lib.io.motors.outputs.control;

import org.austincans.lib.io.motors.outputs.ArbitraryFeedforward;

import edu.wpi.first.units.measure.AngularVelocity;

public class VelocityControlInputs {
    protected AngularVelocity velocity;
    protected VelocityControlType controlType;
    protected ControlSlot controlSlot;
    protected ArbitraryFeedforward arbitraryFeedforward;

    public AngularVelocity getVelocity() {
        return velocity;
    }
    public VelocityControlType getControlType() {
        return controlType;
    }
    public ControlSlot getControlSlot() {
        return controlSlot;
    }
    public ArbitraryFeedforward getArbitraryFeedforward() {
        return arbitraryFeedforward;
    }
}