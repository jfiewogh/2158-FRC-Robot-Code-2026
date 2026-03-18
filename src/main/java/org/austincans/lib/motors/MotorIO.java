package org.austincans.lib.motors;

import java.util.List;

import org.austincans.lib.subsystems.selfcheck.HardwareFault;

public interface MotorIO {
    /** 
     * Includes values that are common for all motors. 
     * Values that are not common should be separated into other records, such as SensorIORecords. 
     */
    public class MotorIORecords {
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

    /**
     * It is plausible not all motors have a neutral/idle mode.
     * If true, then MotorIORequests should not have a NeutralMode.
     * You have a class that extends MotorIORequests for motors that have neutral/idle mode.
     */
    public enum NeutralMode {
        BRAKE,
        COAST
    }

    public enum MotorIORequestMode {
        VOLTAGE,
        VELOCITY,
        POSITION
    }

    public class MotorIORequests {
        public MotorIORequestMode requestMode;
        public double voltage;
        public double velocityRadPerSec;
        public double positionRad;
        public NeutralMode neutralMode;

        public void setVoltage(double voltage) {
            requestMode = MotorIORequestMode.VOLTAGE;
            this.voltage = voltage;
        }
        public void setVelocity(double velocityRadPerSec) {
            requestMode = MotorIORequestMode.VELOCITY;
            this.velocityRadPerSec = velocityRadPerSec;
        }
        public void setPosition(double positionRad) {
            requestMode = MotorIORequestMode.POSITION;
            this.positionRad = positionRad;
        }
        public void setNeutralMode(NeutralMode neutralMode) {
            this.neutralMode = neutralMode;
        }
    }

    public void updateRecords(MotorIORecords records);

    public void applyRequests(MotorIORequests requests);
}
