package org.austincans.lib.motors;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
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
        
        public double getVelocity(AngularVelocityUnit unit){
            return this.velocity.in(unit);
        }
        public double getPosition(AngleUnit unit){
            return this.position.in(unit);
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
        public AngleUnit positionMode;
        public AngularVelocityUnit velocityMode;
        public Voltage voltage;
        public AngularVelocity velocity;
        public Angle position;
        public NeutralMode neutralMode;

        public void setVoltage(double voltage) {
            mode = MotorIOOutputMode.VOLTAGE;
            this.voltage = Volts.of(voltage);
        }
        public void setVelocity(double velocity, AngularVelocityUnit unit) {
            mode = MotorIOOutputMode.VELOCITY;
            this.velocity = unit.of(velocity);
            this.velocityMode = unit;
        }
        public void setPosition(double position, AngleUnit unit) {
            mode = MotorIOOutputMode.POSITION;
            this.position = unit.of(position);
            this.positionMode = unit;
        }
        public void setNeutralMode(NeutralMode neutralMode) {
            this.neutralMode = neutralMode;
        }
    }

    public void updateInputs(MotorIOInputs inputs);

    public void applyOutputs(MotorIOOutputs outputs);
}
