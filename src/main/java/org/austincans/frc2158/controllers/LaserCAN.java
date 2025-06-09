// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.controllers;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

/*
 * @author  Austin Cans Robotics Team 2158
 */
public class LaserCAN {
  private LaserCan lc;

  public LaserCAN(int can_id) {
    // Initialize the CanBridge TCP server to communicate with the LaserCAN
    CanBridge.runTCP(); // Start the CanBridge TCP server
    this.lc = new LaserCan(can_id);
    // Optionally initialise the settings of the LaserCAN,
    // if you haven't already done so in GrappleHook
    try {
      this.lc.setRangingMode(LaserCan.RangingMode.SHORT);
      this.lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      this.lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException cfe) {
      System.out.println("Configuration failed! " + cfe.getLocalizedMessage());
    }
  }

  public LaserCan.Measurement getMeasurement() {
    LaserCan.Measurement measurement = this.lc.getMeasurement();
    return measurement;
  }

  // During Robot Periodic, measure and report.
  public LaserCan.Measurement LaserCanPeriodic() {
    LaserCan.Measurement measurement = getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println(
          "Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value
      // or an unreliable measurement.
    }
    return measurement;
  }
}
