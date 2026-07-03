package org.austincans.lib.io.sensors.encoders.inputs;

import org.austincans.lib.io.Inputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class AbsoluteEncoderIOInputs implements Inputs {
    public boolean connected;
    public Angle absolutePosition;
    public Angle position;
    public AngularVelocity velocity; 
}
