// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.math;

import org.austincans.frc2158.util.viz.Drawable;
import org.littletonrobotics.junction.Logger;

/**
 * Result type for {@link SeparatingAxis}. This class is used to store the penetration depth and
 * direction between two colliding shapes. It is used in collision detection algorithms to determine
 * how much two shapes overlap and in which direction they should be separated.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class Penetration implements Drawable {

  private final String name;

  /** Result type for {@link SeparatingAxis}. */
  public Penetration(String name) {
    this.name = name;
  }

  private double xDir = 0.0;
  private double yDir = 0.0;
  private double depth = 0.0;

  /** Set penetration direction. */
  public void setNormal(double x, double y) {
    this.xDir = x;
    this.yDir = y;
  }

  /** Set penetration depth. */
  public void setDepth(double depth) {
    this.depth = depth;
  }

  /** Get penetration direction x component. */
  public double getXDir() {
    return xDir;
  }

  /** Get penetration direction y component. */
  public double getYDir() {
    return yDir;
  }

  /** Get penetration depth. */
  public double getDepth() {
    return depth;
  }

  @Override
  public void drawImpl() {
    Logger.recordOutput(name + "/XDir", xDir);
    Logger.recordOutput(name + "/YDir", yDir);
    Logger.recordOutput(name + "/Depth", depth);
  }
}
