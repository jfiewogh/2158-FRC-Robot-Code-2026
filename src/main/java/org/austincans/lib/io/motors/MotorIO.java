package org.austincans.lib.io.motors;

import org.austincans.lib.io.ActuatorIO;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO<InputsType extends MotorIO.MotorIOInputs> extends ActuatorIO<InputsType, MotorIO.MotorIOOutputs> {
    /**
     * Values universal for all motors
     * TODO: Current values are universal only for real motors. Eventually, update to be universal for real and sim.  
     */
    public class MotorIOInputs {
        private Angle position;
        private AngularVelocity velocity;
        private Voltage voltage;
        private Current statorCurrent;
        private Temperature temperature;

        /* Setter methods should only be called by IO */
        void setPosition(Angle position) {
            this.position = position;
        }
        void setVelocity(AngularVelocity velocity) {
            this.velocity = velocity;
        }
        void setVoltage(Voltage voltage) {
            this.voltage = voltage;
        }
        void setStatorCurrent(Current statorCurrent) {
            this.statorCurrent = statorCurrent;
        }
        void setTemperature(Temperature temperature) {
            this.temperature = temperature;
        }

        public Angle getPosition() {
            return position;
        }
        public AngularVelocity getVelocity() {
            return velocity;
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
        private MotorIOOutputMode mode;
        private Voltage voltage;
        private AngularVelocity velocity;
        private Angle position;
        private NeutralMode neutralMode; // TODO: maybe this should be part of an updateConfig method

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

        /* Getter methods should only be called by IO */
        MotorIOOutputMode getMode() {
            return mode;
        }
        Voltage getVoltage() {
            return voltage;
        }
        AngularVelocity getVelocity() {
            return velocity;
        }
        Angle getPosition() {
            return position;
        }
        NeutralMode getNeutralMode() {
            return neutralMode;
        }
    }
}
