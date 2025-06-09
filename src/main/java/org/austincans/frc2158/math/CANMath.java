// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.math;

import edu.wpi.first.math.MathUtil;
import java.util.List;

/*
 * @author Austin Cans Robotics Team 2158
 */
public class CANMath {

  // constants

  static final double sq2p1 = 2.414213562373095048802e0;
  static final double sq2m1 = .414213562373095048802e0;
  static final double p4 = .161536412982230228262e2;
  static final double p3 = .26842548195503973794141e3;
  static final double p2 = .11530293515404850115428136e4;
  static final double p1 = .178040631643319697105464587e4;
  static final double p0 = .89678597403663861959987488e3;
  static final double q4 = .5895697050844462222791e2;
  static final double q3 = .536265374031215315104235e3;
  static final double q2 = .16667838148816337184521798e4;
  static final double q1 = .207933497444540981287275926e4;
  static final double q0 = .89678597403663861962481162e3;
  static final double PIO2 = 1.5707963267948966135E0;
  static final double nan = (0.0 / 0.0);
  public static final double kEpsilon = 1e-12;
  public static final double kGravityInPerSecSq = 386.09;

  // rounds (instead of truncating) a double to specified number of decimal places.
  // For example:
  // round(200.3456, 2); // returns 200.35
  public static double round(double value, int places) {
    if (places < 0) throw new IllegalArgumentException();

    long factor = (long) Math.pow(10, places);
    value = value * factor;
    long tmp = Math.round(value);
    return (double) tmp / factor;
  }

  // reduce
  private static double mxatan(double arg) {
    double argsq, value;

    argsq = arg * arg;
    value = ((((p4 * argsq + p3) * argsq + p2) * argsq + p1) * argsq + p0);
    value = value / (((((argsq + q4) * argsq + q3) * argsq + q2) * argsq + q1) * argsq + q0);
    return value * arg;
  }

  // reduce
  private static double msatan(double arg) {
    if (arg < sq2m1) {
      return mxatan(arg);
    }
    if (arg > sq2p1) {
      return PIO2 - mxatan(1 / arg);
    }
    return PIO2 / 2 + mxatan((arg - 1) / (arg + 1));
  }

  // implementation of atan
  public static double atan(double arg) {
    if (arg > 0) {
      return msatan(arg);
    }
    return -msatan(-arg);
  }

  // implementation of atan2
  public static double atan2(double arg1, double arg2) {
    if (arg1 + arg2 == arg1) {
      if (arg1 >= 0) {
        return PIO2;
      }
      return -PIO2;
    }
    arg1 = atan(arg1 / arg2);
    if (arg2 < 0) {
      if (arg1 <= 0) {
        return arg1 + Math.PI;
      }
      return arg1 - Math.PI;
    }
    return arg1;
  }

  // implementation of asin
  public static double asin(double arg) {
    double temp;
    int sign;

    sign = 0;
    if (arg < 0) {
      arg = -arg;
      sign++;
    }
    if (arg > 1) {
      return nan;
    }
    temp = Math.sqrt(1 - arg * arg);
    if (arg > 0.7) {
      temp = PIO2 - atan(temp / arg);
    } else {
      temp = atan(arg / temp);
    }
    if (sign > 0) {
      temp = -temp;
    }
    return temp;
  }

  // implementation of acos
  public static double acos(double arg) {
    if (arg > 1 || arg < -1) {
      return nan;
    }
    return PIO2 - asin(arg);
  }

  /**
   * Get the difference in angle between two angles.
   *
   * @param from The first angle
   * @param to The second angle
   * @return The change in angle from the first argument necessary to line up with the second.
   *     Always between -Pi and Pi
   */
  public static double getDifferenceInAngleRadians(double from, double to) {
    return boundAngleNegPiToPiRadians(to - from);
  }

  /**
   * Get the difference in angle between two angles.
   *
   * @param from The first angle
   * @param to The second angle
   * @return The change in angle from the first argument necessary to line up with the second.
   *     Always between -180 and 180
   */
  public static double getDifferenceInAngleDegrees(double from, double to) {
    return boundAngleNeg180to180Degrees(to - from);
  }

  public static double boundAngle0to360Degrees(double angle) {
    // Naive algorithm
    while (angle >= 360.0) {
      angle -= 360.0;
    }
    while (angle < 0.0) {
      angle += 360.0;
    }
    return angle;
  }

  public static double boundAngleNeg180to180Degrees(double angle) {
    // Naive algorithm
    while (angle >= 180.0) {
      angle -= 360.0;
    }
    while (angle < -180.0) {
      angle += 360.0;
    }
    return angle;
  }

  public static double boundAngle0to2PiRadians(double angle) {
    // Naive algorithm
    while (angle >= 2.0 * Math.PI) {
      angle -= 2.0 * Math.PI;
    }
    while (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }
    return angle;
  }

  public static double boundAngleNegPiToPiRadians(double angle) {
    // Naive algorithm
    while (angle >= Math.PI) {
      angle -= 2.0 * Math.PI;
    }
    while (angle < -Math.PI) {
      angle += 2.0 * Math.PI;
    }
    return angle;
  }

  /** Limits the given input to the given magnitude. */
  public static double limit(double v, double maxMagnitude) {
    return limit(v, -maxMagnitude, maxMagnitude);
  }

  public static double limit(double v, double min, double max) {
    return Math.min(max, Math.max(min, v));
  }

  public static boolean inRange(double v, double maxMagnitude) {
    return inRange(v, -maxMagnitude, maxMagnitude);
  }

  /** Checks if the given input is within the range (min, max), both exclusive. */
  public static boolean inRange(double v, double min, double max) {
    return v > min && v < max;
  }

  public static double interpolate(double a, double b, double x) {
    x = limit(x, 0.0, 1.0);
    return a + (b - a) * x;
  }

  public static String joinStrings(final String delim, final List<?> strings) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < strings.size(); ++i) {
      sb.append(strings.get(i).toString());
      if (i < strings.size() - 1) {
        sb.append(delim);
      }
    }
    return sb.toString();
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(int a, int b, int epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
    boolean result = true;
    for (Double value_in : list) {
      result &= epsilonEquals(value_in, value, epsilon);
    }
    return result;
  }

  public static double rpm_to_rads_per_sec(double rpm) {
    return rpm * 2.0 * Math.PI / 60.0;
  }

  public static double rads_per_sec_to_rpm(double rads_per_sec) {
    return rads_per_sec * 60.0 / (2.0 * Math.PI);
  }

  public static double inches_to_meters(double inches) {
    return inches * 0.0254;
  }

  public static double meters_to_inches(double meters) {
    return meters / 0.0254;
  }

  public static double feet_to_meters(double feet) {
    return inches_to_meters(feet * 12.0);
  }

  public static double meters_to_feet(double meters) {
    return meters_to_inches(meters) / 12.0;
  }

  public static double degrees_to_radians(double degrees) {
    return Math.toRadians(degrees);
  }

  public static double radians_to_degrees(double radians) {
    return Math.toDegrees(radians);
  }

  public static boolean withinTolerance(double value, double target, double tolerance) {
    return Math.abs(target - value) <= tolerance;
  }

  public static double modifyAxis(double value, double deadband) {

    return modifyAxis(value, 1, deadband);
  }

  public static double modifyAxis(double value, int exponent, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);

    value = Math.copySign(Math.pow(value, exponent), value);

    return value;
  }

  /** Prevent this class from being instantiated. */
  public CANMath() {}
}
