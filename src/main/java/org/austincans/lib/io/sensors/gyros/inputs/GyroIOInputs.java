package org.austincans.lib.io.sensors.gyros.inputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOInputs {
    public boolean connected;
    public Angle yawPosition;
    public Angle pitchPosition;
    public Angle rollPosition;
    public AngularVelocity yawVelocity;
    public AngularVelocity pitchVelocity;
    public AngularVelocity rollVelocity;
}
