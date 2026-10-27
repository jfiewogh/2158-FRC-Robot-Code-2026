package org.austincans.lib.io.motors.outputs.control;

import org.austincans.lib.io.motors.outputs.ArbitraryFeedforward;
import org.austincans.lib.io.motors.outputs.Feedforward;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class PositionControlInputs {
    protected Angle position;
    protected PositionControlType controlType;
    protected ControlSlot controlSlot;
    protected AngularVelocity velocity;
    protected Feedforward arbitraryFeedforward = new ArbitraryFeedforward();

    public Angle getPosition() {
        return position;
    }
    public PositionControlType getControlType() {
        return controlType;
    }
    public ControlSlot getControlSlot() {
        return controlSlot;
    }
    public AngularVelocity getVelocity() {
        return velocity;
    }
    public ArbitraryFeedforward getArbitraryFeedforward() {
        return arbitraryFeedforward;
    }
}