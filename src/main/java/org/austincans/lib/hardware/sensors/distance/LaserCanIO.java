package org.austincans.lib.hardware.sensors.distance;
import static edu.wpi.first.units.Units.Millimeters;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class LaserCanIO implements DistanceSensorIO {
    private final LaserCan laserCan;

    public LaserCanIO(int canId, RangingMode mode, RegionOfInterest regionOfInterest, TimingBudget timingBudget) {
        laserCan = new LaserCan(canId);
        try {
            laserCan.setRangingMode(mode);
            laserCan.setRegionOfInterest(regionOfInterest);
            laserCan.setTimingBudget(timingBudget);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }   
    }

    @Override
    public void updateInputs(DistanceSensorIOInputs inputs) {
        inputs.setDistance(Millimeters.of(laserCan.getMeasurement().distance_mm));
    } 
}
