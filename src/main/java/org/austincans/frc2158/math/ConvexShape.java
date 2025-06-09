// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.math;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a convex shape for the purpose of separating axis solving.
 *
 * @author Austin Cans Robotics Team 2158
 */
public interface ConvexShape {

  /** Get the {@link Axis axes} of this {@link ConvexShape}. */
  public Axis[] getAxes();

  /** Get the {@link Interval} of this {@link ConvexShape} projected onto the given {@link Axis}. */
  public Interval project(Axis axis);

  /** Get the center of this {@link ConvexShape}. */
  public Translation2d getCenter();
}
