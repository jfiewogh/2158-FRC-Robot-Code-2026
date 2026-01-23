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
// Updated on Friday January 23, 2026 at 11:47:19 CST

package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.Arena;

public interface Arena2026 extends Arena {
  boolean isActive(boolean isBlue);

  void addPieceWithVariance(
      Translation2d piecePose,
      Rotation2d yaw,
      Distance height,
      LinearVelocity speed,
      Angle pitch,
      double xVariance,
      double yVariance,
      double yawVariance,
      double speedVariance,
      double pitchVariance);

  void outpostDump(boolean isBlue);

  void outpostThrowForGoal(boolean isBlue);

  void outpostThrow(boolean isBlue, Rotation2d throwYaw, Angle throwPitch, LinearVelocity speed);
}
