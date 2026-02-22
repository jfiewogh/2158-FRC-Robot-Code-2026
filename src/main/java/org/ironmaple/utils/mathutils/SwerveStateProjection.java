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
// Updated on Saturday February 21, 2026 at 20:04:14 CST

package org.ironmaple.utils.mathutils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveStateProjection {
  /**
   * Projects the swerve module speed onto the direction of the current swerve facing.
   *
   * @param swerveSpeed The current speed and direction of the swerve module.
   * @param currentSwerveFacing The desired direction to project onto.
   * @return The projected speed in the direction of currentSwerveFacing.
   */
  public static double project(SwerveModuleState swerveSpeed, Rotation2d currentSwerveFacing) {
    // Get the angle of the swerve module's current direction
    Rotation2d swerveModuleAngle = swerveSpeed.angle;

    // Calculate the cosine of the angle difference between swerve module direction and the desired
    // direction
    double cosTheta = Math.cos(swerveModuleAngle.minus(currentSwerveFacing).getRadians());

    // Scale the speed by the cosine value to get the projection
    return swerveSpeed.speedMetersPerSecond * cosTheta;
  }
}
