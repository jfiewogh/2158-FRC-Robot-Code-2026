package org.austincans.lib.io.sensors.distance;

import org.austincans.lib.io.DeviceIO;
import edu.wpi.first.units.measure.Distance;

public interface DistanceSensorIO<InputsType extends DistanceSensorIO.DistanceSensorIOInputs> extends DeviceIO<InputsType> {
    public static class DistanceSensorIOInputs {
        public Distance distance;
    }
}