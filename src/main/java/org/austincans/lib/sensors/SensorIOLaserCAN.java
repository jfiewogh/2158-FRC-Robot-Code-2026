package org.austincans.lib.sensors;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class SensorIOLaserCAN implements SensorIO {
    private LaserCan laserCan;

    public SensorIOLaserCAN(int canId, RangingMode mode, RegionOfInterest regionOfInterest, TimingBudget timingBudget) {
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
    public double getMeasurement() {
        return laserCan.getMeasurement().distance_mm;
    }

    @Override
    public void updateInputs(SensorIOInputs inputs) {
        inputs.measurementMillimeters = getMeasurement();
    } 
}
