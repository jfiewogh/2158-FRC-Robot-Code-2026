package org.austincans.lib.hardware.sensors.distance;

import org.austincans.lib.hardware.HardwareIO;

import edu.wpi.first.units.measure.Distance;

public interface DistanceSensorIO<InputsType extends DistanceSensorIO.DistanceSensorIOInputs> extends HardwareIO<InputsType> {
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