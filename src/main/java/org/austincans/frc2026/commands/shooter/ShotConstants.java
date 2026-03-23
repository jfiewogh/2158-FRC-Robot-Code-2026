package org.austincans.frc2026.commands.shooter;

import org.austincans.frc2026.FieldConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/** This is specifically for the 2026 robot */
public class ShotConstants {
    /* These are used for shooting into the hub */
    private static InterpolatingTreeMap<Double, Rotation2d> scoringHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static InterpolatingDoubleTreeMap scoringFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap scoringTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    /* These are used for shuttling (landing z = 0) */
    private static InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static InterpolatingDoubleTreeMap passingFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap passingTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    /* Add empirical data 
     * key: distance from target in meters
    */
    static {
        scoringHoodAngleMap.put(0.0, Rotation2d.fromRadians(Math.PI / 2));
        scoringHoodAngleMap.put(FieldConstants.fieldLength / 6, Rotation2d.fromRadians(7 * Math.PI / 18));
        scoringHoodAngleMap.put(FieldConstants.fieldLength / 3, Rotation2d.fromRadians(Math.PI / 4));
        scoringFlywheelSpeedMap.put(0.0, 4000.0);
        scoringFlywheelSpeedMap.put(5.0, 10000.0);
        scoringTimeOfFlightMap.put(0.0, 1.0);
        scoringTimeOfFlightMap.put(1.0, 1.0);
        scoringTimeOfFlightMap.put(2.0, 1.0);
        scoringTimeOfFlightMap.put(3.0, 1.0);
        scoringTimeOfFlightMap.put(4.0, 1.0);

        passingHoodAngleMap.put(5.0, Rotation2d.fromDegrees(0));
        passingHoodAngleMap.put(10.0, Rotation2d.fromDegrees(0));
        scoringFlywheelSpeedMap.put(0.0, 10000.0);
        scoringFlywheelSpeedMap.put(5.0, 10000.0);
        passingTimeOfFlightMap.put(0.0, 1.0);
        passingTimeOfFlightMap.put(1.0, 1.0);
        passingTimeOfFlightMap.put(2.0, 1.0);
        passingTimeOfFlightMap.put(3.0, 1.0);
        passingTimeOfFlightMap.put(4.0, 1.0);
    }
}
