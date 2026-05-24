package org.austincans.lib.io.sensors.distance;

import org.austincans.lib.io.DeviceIO;
import edu.wpi.first.units.measure.Distance;

public interface DistanceSensorIO<InputsType extends DistanceSensorIO.DistanceSensorIOInputs> extends DeviceIO<InputsType> {
    public class DistanceSensorIOInputs {
        private Distance distance;

        void setDistance(Distance distance) {
            this.distance = distance;
        }

        public Distance getDistance() {
            return distance;
        }
    }
}