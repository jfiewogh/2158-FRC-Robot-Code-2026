package org.austincans.lib.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO {
    /** 
     * Includes values that are common for all motors. 
     * Values that are not common should be separated into other records, such as SensorIORecords. 
     */
    public class MotorIOInputs {
        public boolean connected = false;
        public Angle position;
        public AngularVelocity velocity;
        public Voltage voltage;
        public double supplyCurrentAmps;
        public double statorCurrentAmps;
        public double torqueCurrentAmps;
        public double temperatureCelsius;
        
        public double getVelocity(){
            return velocity.baseUnitMagnitude();
        }
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

    public enum MotorIOOutputMode {
        VOLTAGE,
        VELOCITY,
        POSITION
    }

    public class MotorIOOutputs {
        public MotorIOOutputMode mode;
        public Voltage voltage;
        public AngularVelocity velocity;
        public Angle position;
        public NeutralMode neutralMode;

        public void setVoltage(Voltage voltage) {
            mode = MotorIOOutputMode.VOLTAGE;
            this.voltage = voltage;
        }
        public void setVelocity(AngularVelocity velocity) {
            mode = MotorIOOutputMode.VELOCITY;
            this.velocity = velocity;
        }
        public void setPosition(Angle position) {
            mode = MotorIOOutputMode.POSITION;
            this.position = position;
        }
        public void setNeutralMode(NeutralMode neutralMode) {
            this.neutralMode = neutralMode;
        }
        public double getVeolcity(){
            return velocity.baseUnitMagnitude();
        }
    }

    public void updateInputs(MotorIOInputs inputs);

    public void applyOutputs(MotorIOOutputs outputs);
}
