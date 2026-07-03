package org.austincans.lib.io.sensors.distance;

import static edu.wpi.first.units.Units.Millimeters;

import org.austincans.lib.io.Inputs;
import org.austincans.lib.io.sensors.distance.inputs.LaserCANIOInputs;
import org.austincans.lib.io.sensors.distance.inputs.MeasurementStatus;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class LaserCANIO implements DistanceSensorIO {
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
  public void updateInputs(Inputs inputs) {
    LaserCANIOInputs laserCANInputs = (LaserCANIOInputs) inputs;

    Measurement measurement = laserCan.getMeasurement();
    laserCANInputs.measurementStatus = switch (measurement.status) {
      case 0 -> MeasurementStatus.VALID;
      case 1 -> MeasurementStatus.NOISE_ISSUE;
      case 2 -> MeasurementStatus.WEAK_SIGNAL;
      case 4 -> MeasurementStatus.OUT_OF_BOUNDS;
      case 7 -> MeasurementStatus.WRAPAROUND;
      default -> MeasurementStatus.OUT_OF_BOUNDS;
    };
    laserCANInputs.distance = Millimeters.of(measurement.distance_mm);
  }
}
