package org.austincans.lib.subsystems.elevator;

import org.austincans.lib.configs.MotorConfigSpark;

import com.revrobotics.spark.config.SparkFlexConfig;

public class ElevatorConfig {
    public MotorConfigSpark motorConfigSpark;
    public SparkFlexConfig sparkFlexConfig;
    public static final double maxVelocityMetersPerSec = 3;
    public static final double maxAccelerationMetersPerSecSq = 3;

    // 13 rotations of motor results in one rotation
    public static final double gearRatio = 13.0;

    public static final double minHeightMeters = 0;
    public static final double maxHeightMeters = 2;

    public static final double drumRadius = 0.0254;

    public static final double minVoltage = -7;
    public static final double maxVoltage = 12;

    public static final double pidTolerance = 0.05;

    public static final int canID = 0;

    public static final boolean enableStatorLimit = true;

    public static final double statorCurrentLimit = 40;

    public static final boolean enableSupplyLimit = false;

    public static final double supplyCurrentLimit = 40;

    // these values are for 2025 only so change it if you need to
    public static enum ElevatorState {
        GROUND(0),
        L1(0.25),
        L2(0.6),
        L3(1.02),
        L4(1.73562);


        private double setpointMeters;

        private ElevatorState(double setpointMeters) {
            this.setpointMeters = setpointMeters;
        }

        public double getSetpointMeters() {
            return setpointMeters;
        }
    }

    // these may be wrong
    public static final double[] stageHeights = new double[] {
            0.662178, // Stage 1
            0.655828, // Stage 2
            0.567614 // Stage 3
    };
}
