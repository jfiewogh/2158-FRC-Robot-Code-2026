package org.austincans.lib.hardware.motors.configs;

public class SparkConfig extends MotorConfig {
   public double allowedClosedLoopErrorRad; // after gearing
   public boolean isBrushless;
   public int smartCurrentStallLimitAmps;
}
