package org.austincans.lib.io.sensors.distance;

import static edu.wpi.first.units.Units.Millimeters;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class LaserCANIO implements DistanceSensorIO<LaserCANIO.LaserCanIOInputs> {
  private final LaserCan laserCan;

  public LaserCANIO(int canId, RangingMode mode, RegionOfInterest regionOfInterest, TimingBudget timingBudget) {
    CanBridge.runTCP();
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
  public void updateInputs(LaserCanIOInputs inputs) {
    Measurement measurement = laserCan.getMeasurement();

    inputs.measurementStatus = switch (measurement.status) {
      case 0 -> MeasurementStatus.VALID;
      case 1 -> MeasurementStatus.NOISE_ISSUE;
      case 2 -> MeasurementStatus.WEAK_SIGNAL;
      case 4 -> MeasurementStatus.OUT_OF_BOUNDS;
      case 7 -> MeasurementStatus.WRAPAROUND;
      default -> MeasurementStatus.OUT_OF_BOUNDS;
    };
    
    inputs.distance = Millimeters.of(measurement.distance_mm);
  }

  public enum MeasurementStatus {
    VALID,
    NOISE_ISSUE,
    WEAK_SIGNAL,
    OUT_OF_BOUNDS,
    WRAPAROUND,
  }

  public class LaserCanIOInputs extends DistanceSensorIO.DistanceSensorIOInputs {
    public MeasurementStatus measurementStatus;
  }
}
