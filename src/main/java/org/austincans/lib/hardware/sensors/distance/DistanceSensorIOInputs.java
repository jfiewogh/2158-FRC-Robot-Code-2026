package org.austincans.lib.hardware.sensors.distance;

import edu.wpi.first.units.measure.Distance;

public class DistanceSensorIOInputs {
    private Distance distance;

    void setDistance(Distance distance) {
        this.distance = distance;
    }

    public Distance getDistance() {
        return distance;
    }
}