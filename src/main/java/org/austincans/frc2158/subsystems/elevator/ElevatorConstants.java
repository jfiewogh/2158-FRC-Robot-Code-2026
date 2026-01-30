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
//
// Updated on Thursday January 29, 2026 at 19:31:22 CST

package org.austincans.frc2158.subsystems.elevator;

import org.austincans.frc2158.util.ConfigUtil;

public class ElevatorConstants {
  public static final double maxVelocityMetersPerSec =
      ConfigUtil.getConfigDouble("ELEVATOR_MAX_VELOCITY", 0);
  public static final double maxAccelerationMetersPerSecSq =
      ConfigUtil.getConfigDouble("ELEVATOR_MAX_ACCELERATION", 0);

  // 13 rotations of motor results in one rotation
  public static final double gearRatio = ConfigUtil.getConfigDouble("ELEVATOR_RATIO", 0);
  ;

  public static final double minHeightMeters = ConfigUtil.getConfigDouble("ELEVATOR_MIN_HEIGHT", 0);
  public static final double maxHeightMeters = ConfigUtil.getConfigDouble("ELEVATOR_MAX_HEIGHT", 0);

  public static final double drumRadius =
      ConfigUtil.getConfigDouble("ELEVATOR_DRUM_RADIUS", 0); // Meters

  public static final double minVoltage = -7;

  public static final double pidTolerance = 0.05;
  public static final double elevatorP = ConfigUtil.getConfigDouble("ELEVATOR_P", 0);
  public static final double elevatorI = ConfigUtil.getConfigDouble("ELEVATOR_I", 0);
  public static final double elevatorD = ConfigUtil.getConfigDouble("ELEVATOR_D", 0);
  public static final double elevatorS = ConfigUtil.getConfigDouble("ELEVATOR_S", 0);
  public static final double elevatorG = ConfigUtil.getConfigDouble("ELEVATOR_G", 0);
  public static final double elevatorV = ConfigUtil.getConfigDouble("ELEVATOR_V", 0);
  public static final double elevatorA = ConfigUtil.getConfigDouble("ELEVATOR_A", 0);

  public static final int canID = ConfigUtil.getConfigInt("ELEVATOR_CAN_ID", 0);

  public static final boolean brakeMode = ConfigUtil.getConfigBoolean("ELEVATOR_BRAKE_MODE", false);

  public static enum ElevatorState {
    EXAMPLE_SETPOINT_1(0),
    EXAMPLE_SETPOINT_2(0.25);

    private double setpointMeters;

    private ElevatorState(double setpointMeters) {
      this.setpointMeters = setpointMeters;
    }

    public double getSetpointMeters() {
      return setpointMeters;
    }
  }
}
