package org.austincans.lib.subsystems.drive;

public class DriveConfig {
    public double robotMassKg;
    public double robotMomentOfInertiaKgMetersSq;
    public double maxDriveSpeedMetersPerSec;
    public double maxOmegaSpeedRadPerSec;
    /** The distance between the left and right motors in meters */
    public double trackWidthMeters;
    /** The distance between the front and back motors in meters */
    public double wheelbaseMeters;
    /** This assumes all wheels are identical. If not, you can make multiple wheelConfigs. */
    public WheelConfig wheelConfig;
}
