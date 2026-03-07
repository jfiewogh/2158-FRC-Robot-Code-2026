package org.austincans.lib.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
    @AutoLog
    public class SensorIOInputs{
        double measurementMillimeters;
    }

    public double getMeasurement();

    public void updateInputs(SensorIOInputs inputs);
}