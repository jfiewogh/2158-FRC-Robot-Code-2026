package org.austincans.lib.io.motors.outputs.control;

import com.revrobotics.spark.SparkBase.ControlType;

public enum SparkVelocityControlType implements VelocityControlType {
    STANDARD(ControlType.kVelocity),
    MAXMOTION(ControlType.kMAXMotionVelocityControl);

    private final ControlType controlType;

    private SparkVelocityControlType(ControlType controlType) {
        this.controlType = controlType;
    }

    public ControlType get() {
        return controlType;
    }
}