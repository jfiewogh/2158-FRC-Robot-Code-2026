package org.austincans.lib.motors;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
    /** Includes values that are common for all motors. Values that are not common are separated into SensorIOs. */
    @AutoLog
    public class MotorIOInputs {
        public boolean connected = false;
        public double positionRad;
        public double velocityRadPerSec;
        public double appliedVolts;
        public double supplyCurrentAmps;
        public double statorCurrentAmps;
        public double torqueCurrentAmps;
        public double temperatureCelsius;
    }    

    public void updateInputs(MotorIOInputs inputs);

    public void setVoltage(double voltage);

    public void setVelocity(double velocityRadPerSec);

    public void setPosition(double positionRad);
}
