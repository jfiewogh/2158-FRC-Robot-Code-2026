package org.austincans.lib.motors;

import java.util.List;

import org.austincans.lib.subsystems.selfcheck.HardwareFault;

public interface MotorIO {
    /** Includes values that are common for all motors. Values that are not common are separated into SensorIOs. */
    public class MotorIOInputs {
        public boolean connected = false;
        public double positionRad;
        public double velocityRadPerSec;
        public double appliedVolts;
        public double supplyCurrentAmps;
        public double statorCurrentAmps;
        public double torqueCurrentAmps;
        public double temperatureCelsius;
        public List<HardwareFault> faults;
    }    

    public void updateInputs(MotorIOInputs inputs);

    public void setVoltage(double voltage);

    public void setVelocity(double velocityRadPerSec);

    public void setPosition(double positionRad);
}
