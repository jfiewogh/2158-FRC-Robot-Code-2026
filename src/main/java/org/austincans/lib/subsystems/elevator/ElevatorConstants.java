// Copyright (c) 2026 the ausTIN CANs
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

package org.austincans.lib.subsystems.elevator;

public class ElevatorConstants {
  public static final double maxVelocityMetersPerSec = 3;
  public static final double maxAccelerationMetersPerSecSq = 3;

  // 13 rotations of motor results in one rotation
  public static final double gearRatio = 13.0;

  public static final double minHeightMeters = 0;
  public static final double maxHeightMeters = 2;

  public static final double drumRadius = 0.0254;

  public static final double minVoltage = -7;

  public static final double pidTolerance = 0.05;

  public static enum ElevatorState {
    GROUND(0),
    L1(0.25),
    L2(0.6),
    L3(1.02),
    L4(1.73562);

    private double setpointMeters;

    private ElevatorState(double setpointMeters) {
      this.setpointMeters = setpointMeters;
    }

    public double getSetpointMeters() {
      return setpointMeters;
    }
  }

  // these may be wrong
  public static final double[] stageHeights =
      new double[] {
        0.662178, // Stage 1
        0.655828, // Stage 2
        0.567614 // Stage 3
      };
}
