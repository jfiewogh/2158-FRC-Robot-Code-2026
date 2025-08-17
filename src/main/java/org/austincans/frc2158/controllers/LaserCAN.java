// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// oooooooooooo ooooooooo.     .oooooo.        ooooooooo.              .o8                     .   
// `888'     `8 `888   `Y88.  d8P'  `Y8b       `888   `Y88.           "888                   .o8   
//  888          888   .d88' 888                888   .d88'  .ooooo.   888oooo.   .ooooo.  .o888oo 
//  888oooo8     888ooo88P'  888                888ooo88P'  d88' `88b  d88' `88b d88' `88b   888   
//  888    "     888`88b.    888                888`88b.    888   888  888   888 888   888   888   
//  888          888  `88b.  `88b    ooo        888  `88b.  888   888  888   888 888   888   888 . 
// o888o        o888o  o888o  `Y8bood8P'       o888o  o888o `Y8bod8P'  `Y8bod8P' `Y8bod8P'   "888" 
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// Updated on Sunday August 17, 2025 at 08:59:14 CDT

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
