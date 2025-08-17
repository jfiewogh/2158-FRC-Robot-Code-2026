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
// Updated on Sunday August 17, 2025 at 08:59:14 CDT

package org.austincans.frc2158.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.austincans.frc2158.util.viz.Drawable;
import org.littletonrobotics.junction.Logger;

/**
 * Rotating Rectangle Shape This class represents a rectangle in 2D space that can be rotated and
 * projected onto an axis. It implements the ConvexShape interface, which allows it to be used in
 * collision detection algorithms. The rectangle is defined by its length, width, and pose (position
 * and orientation) in space. The class provides methods to get the axes of the rectangle, project
 * it onto an axis, and update its vertices. It also includes a method to draw the rectangle using a
 * logger.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class Rectangle implements ConvexShape, Drawable {

  public final double width;
  public final double length;

  private final Axis[] axes;
  private final Translation2d[] vertices;

  private Pose2d pose;

  private final String name;

  /** Rotating Rectangle Shape */
  public Rectangle(String name, Pose2d pose, double length, double width) {
    this.name = name;
    this.pose = pose;
    this.width = width;
    this.length = length;
    this.axes = new Axis[] {new Axis(1, 0), new Axis(1, 0)};
    this.vertices = new Translation2d[5];
  }

  @Override
  public Axis[] getAxes() {
    axes[0].setFromRotation(pose.getRotation());
    axes[1].setFromRotation(pose.getRotation().plus(Rotation2d.kCW_90deg));
    return axes;
  }

  @Override
  public Interval project(Axis axis) {
    updateVertices();
    return axis.project(vertices);
  }

  private void updateVertices() {
    vertices[0] =
        pose.getTranslation()
            .plus(new Translation2d(length / 2.0, width / 2.0).rotateBy(pose.getRotation()));
    vertices[1] =
        pose.getTranslation()
            .plus(new Translation2d(-length / 2.0, width / 2.0).rotateBy(pose.getRotation()));
    vertices[2] =
        pose.getTranslation()
            .plus(new Translation2d(-length / 2.0, -width / 2.0).rotateBy(pose.getRotation()));
    vertices[3] =
        pose.getTranslation()
            .plus(new Translation2d(length / 2.0, -width / 2.0).rotateBy(pose.getRotation()));
    vertices[4] = vertices[0];
  }

  @Override
  public Translation2d getCenter() {
    return pose.getTranslation();
  }

  /** Override rectangle pose. */
  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  @Override
  public void drawImpl() {
    updateVertices();
    Logger.recordOutput(name, vertices);
  }
}
