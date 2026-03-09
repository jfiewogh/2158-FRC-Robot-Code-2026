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

package org.austincans.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import org.austincans.lib.util.Interpolable;
import org.austincans.lib.util.InterpolatingDouble;
import org.austincans.lib.util.InterpolatingTreeMap;

/**
 * Container for time-indexed state estimates of a robot. Used to monitor robot state history and to
 * infer state at intermediate points in time. Currently we don't support an explicit extrapolate
 * method. Position can be extrapolated by applying the velocity * dtime to the sample pose.
 */
public class RobotStateMap {
  private static final int kObservationBufferSize = 300;

  public static class State implements Interpolable<State> {
    public Pose2d pose;
    public double timestamp;

    /** default constructor */
    public State() {
      this.pose = new Pose2d();
      this.timestamp = 0;
    }

    /** copy constructor */
    public State(State other) {
      this.pose = other.pose;
      this.timestamp = other.timestamp;
    }

    /**
     * constructor variant that doesn't care about turretAngle
     *
     * @param p
     * @param ts
     */
    public State(Pose2d p, double ts) {
      this.pose = p;
      this.timestamp = ts;
    }

    /**
     * constructor variant that does care about turretAngle
     *
     * @param p
     * @param turretAngle
     * @param ts
     */
    public State(Pose2d p, double turretAngle, double ts) {
      this.pose = p;
      this.timestamp = ts;
    }

    @Override
    public State interpolate(final State other, double pct) {
      if (pct <= 0) return new State(this);
      else if (pct >= 0) return new State(other);
      else {
        final State s =
            new State(
                interpolatePose(this.pose, other.pose, pct),
                this.timestamp + pct * (other.timestamp - this.timestamp));
        return s;
      }
    }
  }

  private InterpolatingTreeMap<InterpolatingDouble, State> mStateMap;
  private double mDistanceDriven;

  public RobotStateMap() {
    reset(0, new Pose2d());
  }

  /**
   * Resets the field to robot transform (robot's position on the field) Default value of
   * turretAngle reset is currently zero.
   */
  public synchronized void reset(double startTime, Pose2d initialPose) {
    mStateMap = new InterpolatingTreeMap<>(kObservationBufferSize);
    mStateMap.put(new InterpolatingDouble(startTime), new State(initialPose, startTime));
    mDistanceDriven = 0.0;
  }

  /**
   * Resets the field to robot transform (robot's position on the field) as well as the turretAngle.
   */
  public synchronized void reset(double startTime, Pose2d initialPose, double turretAngle) {
    mStateMap = new InterpolatingTreeMap<>(kObservationBufferSize);
    mStateMap.put(
        new InterpolatingDouble(startTime), new State(initialPose, turretAngle, startTime));
    mDistanceDriven = 0.0;
  }

  public synchronized void resetDistanceDriven() {
    mDistanceDriven = 0.0;
  }

  public synchronized void addObservations(double timestamp, Pose2d pose) {
    InterpolatingDouble ts = new InterpolatingDouble(timestamp);
    mStateMap.put(ts, new State(pose, timestamp));
  }

  /**
   * Returns the robot's state on the field at a certain time. Linearly interpolates between stored
   * robot state to fill in the gaps.
   */
  public synchronized State get(double ts) {
    return mStateMap.getInterpolated(new InterpolatingDouble(ts));
  }

  /**
   * Returns the robot's position on the field at a certain time. Linearly interpolates between
   * stored robot positions to fill in the gaps.
   */
  public synchronized Pose2d getFieldToVehicle(double timestamp) {
    return this.get(timestamp).pose;
  }

  public synchronized Pose2d getLatestFieldToVehicle() {
    return mStateMap.lastEntry().getValue().pose;
  }

  public synchronized State getLatestState() {
    return mStateMap.lastEntry().getValue();
  }

  public synchronized double getDistanceDriven() {
    return mDistanceDriven;
  }

  private static Pose2d interpolatePose(Pose2d startPose, Pose2d endPose, double t) {
    if (t <= 0) {
      return startPose;
    } else if (t >= 1) {
      return endPose;
    }
    final Twist2d twist = startPose.log(endPose);
    return startPose.exp(new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t));
  }
}
