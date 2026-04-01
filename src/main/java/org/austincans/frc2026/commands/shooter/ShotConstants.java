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

package org.austincans.frc2026.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import org.austincans.frc2026.Constants;

/** This is specifically for the 2026 robot */
public class ShotConstants {
  /* These are used for shooting into the hub */
  public static InterpolatingTreeMap<Double, Rotation2d> scoringHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  public static InterpolatingDoubleTreeMap scoringFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap scoringTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  /* These are used for shuttling (landing z = 0) */
  public static InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  public static InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // TODO: make version for sim and for real


  /* Add empirical data
   * key: distance from target in meters
   */
  static {
    if (Constants.currentMode == Constants.Mode.SIM) {
      scoringHoodAngleMap.put(0.0, Rotation2d.fromDegrees(90));
      scoringHoodAngleMap.put(1.0, Rotation2d.fromDegrees(80));
      scoringHoodAngleMap.put(2.0, Rotation2d.fromDegrees(70));
      scoringHoodAngleMap.put(3.0, Rotation2d.fromDegrees(60));
      scoringHoodAngleMap.put(4.0, Rotation2d.fromDegrees(50));
      scoringHoodAngleMap.put(5.0, Rotation2d.fromDegrees(45));
      scoringFlywheelSpeedMap.put(0.0, 1000.0);
      scoringFlywheelSpeedMap.put(1.0, 1110.0);
      scoringFlywheelSpeedMap.put(2.0, 1220.0);
      scoringFlywheelSpeedMap.put(3.0, 1330.0);
      scoringFlywheelSpeedMap.put(4.0, 1430.0);
      scoringFlywheelSpeedMap.put(5.0, 1600.0);
      scoringTimeOfFlightMap.put(0.0, 0.9);
      scoringTimeOfFlightMap.put(1.0, 0.9);
      scoringTimeOfFlightMap.put(2.0, 0.9);
      scoringTimeOfFlightMap.put(3.0, 0.9);
      scoringTimeOfFlightMap.put(4.0, 0.9);
      scoringTimeOfFlightMap.put(5.0, 0.9);

      passingHoodAngleMap.put(0.0, Rotation2d.fromDegrees(90));
      passingHoodAngleMap.put(1.0, Rotation2d.fromDegrees(80));
      passingHoodAngleMap.put(2.0, Rotation2d.fromDegrees(70));
      passingHoodAngleMap.put(3.0, Rotation2d.fromDegrees(60));
      passingHoodAngleMap.put(4.0, Rotation2d.fromDegrees(50));
      passingHoodAngleMap.put(5.0, Rotation2d.fromDegrees(45));
      passingFlywheelSpeedMap.put(0.0, 900.0);
      passingFlywheelSpeedMap.put(1.0, 1000.0);
      passingFlywheelSpeedMap.put(2.0, 1110.0);
      passingFlywheelSpeedMap.put(3.0, 1210.0);
      passingFlywheelSpeedMap.put(4.0, 1310.0);
      passingFlywheelSpeedMap.put(5.0, 1450.0);
      passingTimeOfFlightMap.put(0.0, 0.9);
      passingTimeOfFlightMap.put(1.0, 0.9);
      passingTimeOfFlightMap.put(2.0, 0.9);
      passingTimeOfFlightMap.put(3.0, 0.9);
      passingTimeOfFlightMap.put(4.0, 0.9);
      passingTimeOfFlightMap.put(5.0, 0.9);
    } else {
      scoringHoodAngleMap.put(1.5, Rotation2d.fromDegrees(79.1));
      scoringHoodAngleMap.put(3.3, Rotation2d.fromDegrees(79.1));
      scoringHoodAngleMap.put(3.5, Rotation2d.fromDegrees(77.6));
      scoringHoodAngleMap.put(4.0, Rotation2d.fromDegrees(75.8));
      scoringHoodAngleMap.put(4.25, Rotation2d.fromDegrees(74));
      scoringHoodAngleMap.put(4.4, Rotation2d.fromDegrees(72.3));
      scoringHoodAngleMap.put(4.625, Rotation2d.fromDegrees(70.5));
      scoringFlywheelSpeedMap.put(1.5, 2000.0);
      scoringFlywheelSpeedMap.put(2.0, 2800.0);
      scoringFlywheelSpeedMap.put(2.25, 3000.0);
      scoringFlywheelSpeedMap.put(2.527, 3500.0);
      scoringFlywheelSpeedMap.put(3.0, 4000.0);
      scoringFlywheelSpeedMap.put(3.3, 7500.0);
      scoringFlywheelSpeedMap.put(5.0, 7500.0);
      scoringTimeOfFlightMap.put(1.5, 1.25);
      scoringTimeOfFlightMap.put(5.0, 1.25);
    }
  }
}
