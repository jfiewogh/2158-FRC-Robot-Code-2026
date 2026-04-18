package org.austincans.lib.motors;

public interface MotorIO {
    /** 
     * Includes values that are common for all motors. 
     * Values that are not common should be separated into other records, such as SensorIORecords. 
     */
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

    public class MotorIOOutputs {
        public MotorIORequestMode mode;
        public double voltage;
        public double velocityRadPerSec;
        public double positionRad;
        public NeutralMode neutralMode;

        public void setVoltage(double voltage) {
            mode = MotorIORequestMode.VOLTAGE;
            this.voltage = voltage;
        }
        public void setVelocity(double velocityRadPerSec) {
            mode = MotorIORequestMode.VELOCITY;
            this.velocityRadPerSec = velocityRadPerSec;
        }
        public void setPosition(double positionRad) {
            mode = MotorIORequestMode.POSITION;
            this.positionRad = positionRad;
        }
        public void setNeutralMode(NeutralMode neutralMode) {
            this.neutralMode = neutralMode;
        }
    }

    public void updateInputs(MotorIOInputs inputs);

    public void applyOutputs(MotorIOOutputs outputs);
}
