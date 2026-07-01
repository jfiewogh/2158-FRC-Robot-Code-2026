package org.austincans.lib.io.motors.outputs.control;

import com.revrobotics.spark.SparkBase.ControlType;

public enum SparkPositionControlType implements PositionControlType {
    STANDARD(ControlType.kPosition),
    MAXMOTION(ControlType.kMAXMotionPositionControl);

    private final ControlType controlType;

    private SparkPositionControlType(ControlType controlType) {
        this.controlType = controlType;
    }

    public ControlType get() {
        return controlType;
    }
}