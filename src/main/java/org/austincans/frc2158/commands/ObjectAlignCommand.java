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

package org.austincans.frc2158.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers;
import org.austincans.frc2158.subsystems.vision.LimelightHelpers.LimelightTarget_Classifier;

public class ObjectAlignCommand {
  static LimelightTarget_Classifier[] results =
      LimelightHelpers.getLatestResults("").targets_Classifier;
  static LimelightTarget_Classifier closestObject = null;

  public static void alignToClosestObject(Drive drive) {
    if (results.length == 0) {
      return;
    } else {
      closestObject = results[0]; // Use Closest Contour sorting
    }

    double kP_Angular = .035;
    double targetAngularVelocity = closestObject.tx * kP_Angular;

    // convert to radians per second for our drive method
    targetAngularVelocity *= 0; // TODO: Replace with swerve max angular speed
    targetAngularVelocity *= -1.0;

    double kP_Lateral = .1;
    double targetLateralVelocity = closestObject.ty * kP_Lateral;
    targetLateralVelocity *= 0; // TODO: Replace with swerve max lateral speed
    targetLateralVelocity *= -1.0;

    drive.runVelocity(
        new ChassisSpeeds(
            targetLateralVelocity, // Forwards TODO: Replace with Driver Joystick?
            0, // Sideways TODO: Replace with Driver Joystick?
            targetAngularVelocity // Angular
            ));
  }
}
