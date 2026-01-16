package org.austincans.frc2158.util;

public class ShooterMath {

  private static final double GRAVITY = 9.80665;

  // Calculate flywheel velocity given distance and setAngle (turret)
  public static double calculateExitVelocity(
      double distanceMeters,
      double heightDifferenceMeters,
      double hoodAngleRadians) {

    double cos = Math.cos(hoodAngleRadians);
    double tan = Math.tan(hoodAngleRadians);

    double numerator = GRAVITY * distanceMeters * distanceMeters;
    double denominator =
        2 * cos * cos * (distanceMeters * tan - heightDifferenceMeters);

    if (denominator <= 0) return Double.NaN;

    return Math.sqrt(numerator / denominator);
  }

}
