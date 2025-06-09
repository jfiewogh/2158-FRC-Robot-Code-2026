// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.util.viz;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.austincans.frc2158.util.ScoringLocation;

/** Visualization for the whole field */
public class FieldViz implements Drawable {

  /** Put coral on the reef */
  public void scoreCoral(
      Alliance alliance, ScoringLocation.CoralLocation branch, ScoringLocation.Height height) {
    //
  }

  /** Put algae in the barge */
  public void scoreAlgaeInBarge(Alliance alliance, double yPos) {
    //
  }

  /** Put algae in the processor */
  public void scoreAlgaeInProcessor(Alliance alliance, double yPos) {
    //
  }

  @Override
  public void drawImpl() {}
}
