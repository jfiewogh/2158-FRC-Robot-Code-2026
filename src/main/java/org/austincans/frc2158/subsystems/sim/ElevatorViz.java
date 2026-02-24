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
// Updated on Sunday February 22, 2026 at 14:08:53 CST

package org.austincans.frc2158.subsystems.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.austincans.frc2158.subsystems.elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

// Visualizes the elevator stages based on the given position in meters
// This is specifically for the 2025 robot's three-stage elevator
public class ElevatorViz {
  private final String key;

  private double[] stageDisplacements = new double[4];

  public ElevatorViz(String key, double startingHeightMeters) {
    this.key = key;
    updateStageDisplacements(startingHeightMeters);
  }

  public void update(double heightMeters) {
    double delta = heightMeters - getVizHeight();

    updateStageDisplacements(delta);

    logStagePoses();
  }

  private double getVizHeight() {
    double sum = 0;
    for (int i = 0; i <= 3; i++) {
      sum += stageDisplacements[i];
    }
    return sum;
  }

  /** Log the poses of the stages */
  private void logStagePoses() {
    double totalHeight = 0;
    for (int i = 0; i <= 3; i++) {
      totalHeight += stageDisplacements[i];
      Logger.recordOutput(key + "/Stage" + i, new Pose3d(0, 0, totalHeight, Rotation3d.kZero));
    }
  }

  /** Update the stage displacements based on the height delta */
  private void updateStageDisplacements(double delta) {
    int index = 1;
    // If delta is positive
    if (delta > 0) {
      while (delta != 0 && index <= 3) {
        double stageHeight = ElevatorConstants.stageHeights[index - 1];
        if (stageDisplacements[index] != stageHeight) {
          double result = stageDisplacements[index] + delta;
          // delta cannot be negative
          delta = Math.max(0, result - stageHeight);
          // cannot go over stage height
          stageDisplacements[index] = Math.min(result, stageHeight);
        }
        index++;
      }
    }
    // If delta is negative
    else if (delta < 0) {
      while (delta != 0 && index <= 3) {
        if (stageDisplacements[index] != 0) {
          double result = stageDisplacements[index] + delta;
          // delta cannot be positive
          delta = Math.min(0, result);
          // cannot go under 0
          stageDisplacements[index] = Math.max(result, 0);
        }
        index++;
      }
    }
  }
}
