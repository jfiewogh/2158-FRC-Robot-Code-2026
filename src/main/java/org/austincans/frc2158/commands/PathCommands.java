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
// Updated on Thursday February 12, 2026 at 19:07:37 CST

package org.austincans.frc2158.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.drive.DriveConstants;

public class PathCommands {
  // Create the constraints to use while pathfinding
  public static final PathConstraints pathConstraints =
      new PathConstraints(
          DriveConstants.maxLinearSpeed, DriveConstants.maxLinearAcceleration,
          DriveConstants.maxAngularSpeed, DriveConstants.maxAngularAcceleration);

  public static Command followPathToPose(Drive drive, Pose2d pose) {
    return Commands.defer(() -> AutoBuilder.pathfindToPose(pose, pathConstraints), Set.of(drive));
  }
}
