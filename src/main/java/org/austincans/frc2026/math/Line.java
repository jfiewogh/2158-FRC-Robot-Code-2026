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

package org.austincans.frc2026.math;

import edu.wpi.first.math.geometry.Translation2d;
import org.austincans.frc2026.FieldConstants;
import org.austincans.frc2026.util.viz.Drawable;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a line in 2d space. Uses Ax + By + c = 0 representation. This class is used to
 * represent lines in the field coordinate system. It is used to calculate intersections between
 * lines, and to draw lines on the field. It is also used to represent the boundaries of the field.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class Line implements Drawable {

  public double a;
  public double b;
  public double c;
  public String name;

  /** Represents a line in 2d space. Uses Ax + By + c = 0 representation. */
  public Line(String name, double a, double b, double c) {
    this.name = name;
    this.a = a;
    this.b = b;
    this.c = c;
  }

  private static final double minX = -20.0;
  private static final double maxX = FieldConstants.fieldLength + 20.0;
  private static final double minY = -20.0;
  private static final double maxY = FieldConstants.fieldWidth + 20.0;

  @Override
  public void drawImpl() {
    if (a == 0 && b == 0) {
      throw new IllegalArgumentException("a and b cannot simultaneously be zero!");
    } else if (b == 0) {
      double x = -c / a;
      Logger.recordOutput(
          name, new Translation2d[] {new Translation2d(x, minY), new Translation2d(x, maxY)});
    } else if (a == 0) {
      double y = -c / b;
      Logger.recordOutput(
          name, new Translation2d[] {new Translation2d(minX, y), new Translation2d(maxX, y)});
    } else {
      double x1 = minX;
      double y1 = (-c - a * x1) / b;
      if (y1 > maxY) {
        y1 = maxY;
        x1 = (-c - b * y1) / a;
      } else if (y1 < minY) {
        y1 = minY;
        x1 = (-c - b * y1) / a;
      }
      double x2 = maxX;
      double y2 = (-c - a * x2) / b;
      if (y2 > maxY) {
        y2 = maxY;
        x2 = (-c - b * y2) / a;
      } else if (y2 < minY) {
        y2 = minY;
        x2 = (-c - b * y2) / a;
      }
      Logger.recordOutput(
          name, new Translation2d[] {new Translation2d(x1, y1), new Translation2d(x2, y2)});
    }
  }

  /** Get intersection of two lines. Returns null if lines are parallel. */
  public Translation2d intersection(Line other) {
    double d = a * other.b - b * other.a;
    if (d != 0) {
      double dx = -c * other.b + b * other.c;
      double dy = -a * other.c + c * other.a;
      return new Translation2d(dx / d, dy / d);
    } else {
      return null;
    }
  }
}
