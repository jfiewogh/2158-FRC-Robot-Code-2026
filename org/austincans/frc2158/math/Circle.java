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
import org.austincans.frc2158.util.viz.Drawable;
import org.littletonrobotics.junction.Logger;

/* Represents a circle with a known center and radius.
 * @author Austin Cans Robotics Team 2158
 */
public class Circle implements ConvexShape, Drawable {

    public String name;
    private Translation2d center;
    private double radius;

    /** Represents a circle with a known center and radius. */
    public Circle(String name, Translation2d center, double radius) {
        this.name = name;
        this.center = center;
        this.radius = radius;
    }

    private static final int RESOLUTION = 20;

    @Override
    public void drawImpl() {
        Translation2d[] points = new Translation2d[RESOLUTION + 1];
        for (int i = 0; i < RESOLUTION + 1; i++) {
            points[i] =
                    center.plus(new Translation2d(radius, Rotation2d.fromRotations((double) i / (double) RESOLUTION)));
        }
        Logger.recordOutput(name, points);
    }

    private static final Axis[] axes = new Axis[0];

    @Override
    public Axis[] getAxes() {
        return axes;
    }

    @Override
    public Interval project(Axis axis) {
        double c = axis.dot(center);
        return new Interval(c - this.radius, c + this.radius);
    }

    @Override
    public Translation2d getCenter() {
        return center;
    }

    /** Set the circle's center */
    public void setCenter(Translation2d center) {
        this.center = center;
    }

    /** Get the circle's radius */
    public double getRadius() {
        return radius;
    }

    /** Set the circle's radius */
    public void setRadius(double radius) {
        this.radius = radius;
    }

    /** Get if a point is within the boundary of the circle. */
    public boolean contains(Translation2d pt) {
        return pt.minus(center).getNorm() < radius;
    }

    /** Get the signed distance of a point with respect to the circle. */
    public double sdf(Translation2d pt) {
        return pt.minus(center).getNorm() - radius;
    }

    /** Get angle of a point with respect to the circle's center. */
    public Rotation2d getAngle(Translation2d pt) {
        return pt.minus(center).getAngle();
    }

    /** Get point on the circle with a given angle. */
    public Translation2d getVertex(Rotation2d rotation) {
        return getVertex(rotation, 0);
    }

    /** Get point on the circle with a given angle and extra radius. */
    public Translation2d getVertex(Rotation2d rotation, double extra) {
        return center.plus(new Translation2d(radius + extra, rotation));
    }

    /** Get angles for points that are both tangent to the circle and colinear with a given point. */
    public RotationInterval circleTangentAngles(Translation2d p) {
        Translation2d diff = p.minus(center);
        double d = diff.getNorm();
        double det = radius / d;
        if (det > 1.0 || det < -1.0) {
            var angle = getAngle(p);
            return new RotationInterval(angle.minus(Rotation2d.fromDegrees(5)), angle.plus(Rotation2d.fromDegrees(5)));
        }
        double dtheta = Math.acos(det);
        if (dtheta < 0.0) {
            dtheta = -dtheta;
        }

        Rotation2d dAngle = Rotation2d.fromRadians(dtheta);
        return RotationInterval.acute(
                diff.getAngle().plus(dAngle), diff.getAngle().minus(dAngle));
    }
}
