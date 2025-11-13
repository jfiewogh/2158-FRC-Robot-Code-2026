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
// Updated on Thursday November 13, 2025 at 17:43:29 CST

package org.austincans.frc2158.math;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Implementation of Separating Axis Theorem solver. This class is used to determine if two convex
 * shapes are intersecting and to find the minimum translation vector to separate them.
 *
 * @author Austin Cans Robotics Team 2158
 */
public final class SeparatingAxis {

  private SeparatingAxis() {}

  /**
   * Solve the for the separating axis between two convex shape.
   *
   * @param shape1 resolving object. Result contains the transform for this shape to no longer be
   *     penetrating.
   * @param shape2 pseudo-static object. Result assumes this shape does not move.
   * @param penetration out-parameter. Contains the direction and length of the minimum translation
   *     vector.
   * @return {@code true} if {@code shape1} and {@code shape2} are intersecting. If {@code false},
   *     {@code penetration} is unmodified.
   */
  public static boolean solve(ConvexShape shape1, ConvexShape shape2, Penetration penetration) {

    Axis n = null;
    double overlap = Double.MAX_VALUE;

    Axis[] axes1 = shape1.getAxes();
    Axis[] axes2 = shape2.getAxes();

    for (int i = 0; i < axes1.length; i++) {
      Axis axis = axes1[i];
      Interval intervalA = shape1.project(axis);
      Interval intervalB = shape2.project(axis);

      if (!intervalA.overlaps(intervalB)) {
        return false;
      } else {
        double o = intervalA.getOverlap(intervalB);
        // Check for containment
        if (intervalA.contains(intervalB) || intervalB.contains(intervalA)) {
          // if containment exists then get the overlap plus the distance
          // to between the two end points that are the closest
          double max = Math.abs(intervalA.getMax() - intervalB.getMax());
          double min = Math.abs(intervalA.getMin() - intervalB.getMin());
          if (max > min) {
            // if the min differences is less than the max then we need
            // to flip the penetration axis
            axis = axis.unaryMinus();
            o += min;
          } else {
            o += max;
          }
        }

        if (o < overlap) {
          overlap = o;
          n = axis;
        }
      }
    }

    for (int i = 0; i < axes2.length; i++) {
      Axis axis = axes2[i];
      Interval intervalA = shape1.project(axis);
      Interval intervalB = shape2.project(axis);

      if (!intervalA.overlaps(intervalB)) {
        return false;
      } else {
        double o = intervalA.getOverlap(intervalB);
        // Check for containment
        if (intervalA.contains(intervalB) || intervalB.contains(intervalA)) {
          // if containment exists then get the overlap plus the distance
          // to between the two end points that are the closest
          double max = Math.abs(intervalA.getMax() - intervalB.getMax());
          double min = Math.abs(intervalA.getMin() - intervalB.getMin());
          if (max > min) {
            // if the min differences is less than the max then we need
            // to flip the penetration axis
            axis = axis.unaryMinus();
            o += min;
          } else {
            o += max;
          }
        }

        if (o < overlap) {
          overlap = o;
          n = axis;
        }
      }
    }

    Translation2d c1 = shape1.getCenter();
    Translation2d c2 = shape2.getCenter();
    Translation2d cToc = c1.minus(c2);
    if (n.dot(cToc) < 0) {
      n = n.unaryMinus();
    }

    penetration.setNormal(n.getX(), n.getY());
    penetration.setDepth(overlap);

    return true;
  }
}
