package org.austincans.lib.motors;

public enum MotorType {
    // TODO: Maybe create supplier for creating an instance of class with specific parameters
    NEO_550(MotorIOSparkMax.class),
    NEO_VORTEX(MotorIOSparkFlex.class),
    KRAKEN(MotorIOTalonFX.class);

    public final Class<?> motorClass;

    private MotorType(Class<?> motorClass) {
        this.motorClass = motorClass;
    }
}
