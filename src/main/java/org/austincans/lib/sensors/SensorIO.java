package org.austincans.lib.sensors;

public interface SensorIO {
    public class SensorIOInputs{
        public double measurementMillimeters;
    }

    public double getMeasurement();

    public void updateInputs(SensorIOInputs inputs);
}