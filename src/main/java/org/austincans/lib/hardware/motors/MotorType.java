package org.austincans.lib.hardware.motors;

public enum MotorType {
    // TODO: Maybe create supplier for creating an instance of class with specific parameters
    NEO_550(SparkMaxIO.class),
    NEO_VORTEX(SparkFlexIO.class),
    KRAKEN(TalonFXIO.class);

    public final Class<?> motorClass;

    private MotorType(Class<?> motorClass) {
        this.motorClass = motorClass;
    }
}
