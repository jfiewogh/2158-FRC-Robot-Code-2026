package org.austincans.lib.io.motors;

public class SparkConfig extends MotorConfig {
   public double allowedClosedLoopErrorRad; // after gearing
   public boolean isBrushless;
   public int smartCurrentStallLimitAmps;
}
