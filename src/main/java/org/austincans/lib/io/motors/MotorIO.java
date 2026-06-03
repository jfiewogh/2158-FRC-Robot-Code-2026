package org.austincans.lib.io.motors;

import org.austincans.lib.io.ActuatorIO;
import org.austincans.lib.io.motors.MotorConfig.ControlConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO<InputsType extends MotorIO.MotorIOInputs, ConfigType extends MotorConfig<?>> extends ActuatorIO<InputsType, MotorIO.MotorIOOutputs> {
    /**
     * Values universal for all motors
     * TODO: Current values are universal only for real motors. Eventually, update to be universal for real and sim.  
     */
    public static class MotorIOInputs {
        private Angle mechanismPosition;
        private AngularVelocity mechanismVelocity;
        private Voltage voltage;
        private Current statorCurrent;
        private Temperature temperature;

        /* Setter methods should only be called by IO */
        protected void setPosition(Angle position) {
            mechanismPosition = position;
        }
        protected void setVelocity(AngularVelocity velocity) {
            mechanismVelocity = velocity;
        }
        protected void setVoltage(Voltage voltage) {
            this.voltage = voltage;
        }
        protected void setStatorCurrent(Current statorCurrent) {
            this.statorCurrent = statorCurrent;
        }
        protected void setTemperature(Temperature temperature) {
            this.temperature = temperature;
        }

        public Angle getPosition() {
            return mechanismPosition;
        }
        public AngularVelocity getVelocity() {
            return mechanismVelocity;
        }
        public Voltage getVoltage() {
            return voltage;
        }
        public Current getStatorCurrent() {
            return statorCurrent;
        }
        public Temperature getTemperature() {
            return temperature;
        }
    }     
    
    /**
     * It is plausible not all motors have a neutral/idle mode.
     * If true, then MotorIORequests should not have a NeutralMode.
     * You have a class that extends MotorIORequests for motors that have neutral/idle mode.
     */
    public static enum NeutralMode {
        BRAKE,
        COAST
    }

    public static enum MotorIOOutputMode {
        VOLTAGE,
        VELOCITY,
        POSITION
    }

    public static class MotorIOOutputs {
        private MotorIOOutputMode mode;
        private Voltage voltage;
        private AngularVelocity velocity;
        private Angle position;
        private ControlSlot controlSlot;

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
        public void setControlSlot(ControlSlot controlSlot) {
            this.controlSlot = controlSlot;
        }

        /* Getter methods should only be called by IO */
        protected MotorIOOutputMode getMode() {
            return mode;
        }
        protected Voltage getVoltage() {
            return voltage;
        }
        protected AngularVelocity getVelocity() {
            return velocity;
        }
        protected Angle getPosition() {
            return position;
        } 
        protected ControlSlot getControlSlot() {
            return controlSlot;
        }
    }

    // TODO: 1. Complete this method and implementation for SparkIO and TalonIO subclasses
    public static interface PositionControlType {}

    // TODO: 2. Complete this method and implementation for SparkIO and TalonIO subclasses
    public static interface VelocityControlType {}

    /** A control slot represents a possible choice of control gains. */
    public static interface ControlSlot {
        int getIndex();
    }

    public void applyConfigRequests(ConfigType desiredConfig);






    // TODO: 4. Figure out a better way to initialize outputs + clean up code

    /** This should be called in the constructor of the subsystem using this IO */
    public void initializeOutputs(MotorIOOutputs outputs);

    /** TODO: Update this to work with all configs (single motor config instance, instead of part of it) */

    /** TODO: Maybe create a config class containing only values that are tuned/changed
     * (this could possibly be derived from the motor config class) */
    public void updateGains(ControlConfig config, ControlSlot slot);
}
