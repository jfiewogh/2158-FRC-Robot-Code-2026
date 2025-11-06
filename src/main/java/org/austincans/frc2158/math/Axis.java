// Copyright (c) 2025 the ausTIN CANs
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
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

package org.austincans.frc2158.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents an axis. An axis is a direction in 2D space, and is used to project points onto it.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class Axis {

  private double xDir;
  private double yDir;

  /** Create new axis with a given direction. Either xDir or yDir must be non-zero. */
  public Axis(double xDir, double yDir) {
    setDirectionImpl(xDir, yDir);
  }

  /** Create an axis with the same direction as the given rotation. */
  public static Axis fromRotation(Rotation2d rot) {
    return new Axis(rot.getCos(), rot.getSin());
  }

  /** Set axis direction to the same direction as the given rotation. */
  public void setFromRotation(Rotation2d rot) {
    setDirectionImpl(rot.getCos(), rot.getSin());
  }

  /**
   * Get opposite direction. Is equivalent as far as the definition of an axis is concerned, but for
   * implementation purposes, has a negative direction.
   */
  public Axis unaryMinus() {
    return new Axis(-xDir, -yDir);
  }

  /** Get the X direction. */
  public double getX() {
    return xDir;
  }

  /** Get the Y direction. */
  public double getY() {
    return yDir;
  }

  /** Set the direction. Either xDir or yDir must be non-zero. */
  public void setDirection(double xDir, double yDir) {
    setDirectionImpl(xDir, yDir);
  }

  private void setDirectionImpl(double xDir, double yDir) {
    double norm = Math.hypot(xDir, yDir);
    this.xDir = xDir / norm;
    this.yDir = yDir / norm;
  }

  /** Get the dot product between this direction and a given point. */
  public double dot(Translation2d point) {
    return this.xDir * point.getX() + this.yDir * point.getY();
  }

  /** Project multiple points onto this axis. */
  public Interval project(Translation2d[] points) {
    double v = 0.0;
    Translation2d p = points[0];
    double min = this.dot(p);
    double max = min;

    for (int i = 1; i < points.length; i++) {
      p = points[i];
      v = this.dot(p);

      if (v < min) {
        min = v;
      } else if (v > max) {
        max = v;
      }
    }

    return new Interval(min, max);
  }
}
