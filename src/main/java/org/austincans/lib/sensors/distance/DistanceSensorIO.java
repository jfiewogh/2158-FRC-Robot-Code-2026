package org.austincans.lib.sensors.distance;

import edu.wpi.first.units.measure.Distance;

public interface DistanceSensorIO {
    public class DistanceSensorIOInputs{
        public Distance distance;
    }

    public void updateInputs(DistanceSensorIOInputs inputs);
}