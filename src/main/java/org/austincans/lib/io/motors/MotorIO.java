package org.austincans.lib.io.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.austincans.lib.io.ActuatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO<
    InputsType extends MotorIO.MotorIOInputs, 
    ControlSlotType extends MotorIO.ControlSlot,
    VelocityControlTypeType extends MotorIO.VelocityControlType,
    PositionControlTypeType extends MotorIO.PositionControlType,
    ConfigRequestsType extends MotorIO.MotorIOConfigRequests<ConfigType>,
    ConfigType extends MotorConfig
> extends ActuatorIO<
    InputsType, 
    MotorIO.MotorIOOutputs<ControlSlotType, VelocityControlTypeType, PositionControlTypeType>
> {
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

    public static class MotorIOOutputs<
        ControlSlotType extends ControlSlot, 
        VelocityControlTypeType extends VelocityControlType, 
        PositionControlTypeType extends PositionControlType
    > {
        private MotorIOOutputMode mode;
        private Voltage voltage;
        private AngularVelocity velocity;
        private Angle position;
        private ArbitraryFeedforward

        private ControlSlotType controlSlot;
        private VelocityControlTypeType velocityControlType;
        private PositionControlTypeType positionControlType;

        public void setVoltage(Voltage voltage) {
            mode = MotorIOOutputMode.VOLTAGE;
            this.voltage = voltage;
        }

        public void setVelocity(AngularVelocity velocity, VelocityControlTypeType velocityControlType, ControlSlotType controlSlot, Voltage arbitraryFeedforward) {
            mode = MotorIOOutputMode.VELOCITY;
            this.velocity = velocity;
            voltage = arbitraryFeedforward;
            this.controlSlot = controlSlot;
            this.velocityControlType = velocityControlType;
        }
        public void setVelocity(AngularVelocity velocity, VelocityControlTypeType velocityControlType, ControlSlotType controlSlot) {
            setVelocity(velocity, velocityControlType, controlSlot, Volts.of(0));
        }

        public void setPosition(Angle position, PositionControlTypeType positionControlType, ControlSlotType controlSlot, AngularVelocity velocity, Voltage arbitraryFeedforward) {
            mode = MotorIOOutputMode.POSITION;
            this.position = position;
            this.velocity = velocity;
            voltage = arbitraryFeedforward;
            this.controlSlot = controlSlot;
            this.positionControlType = positionControlType;
        }
        public void setPosition(Angle position, PositionControlTypeType positionControlType, ControlSlotType controlSlot, AngularVelocity velocity) {
            setPosition(position, positionControlType, controlSlot, velocity, Volts.of(0));
        }
        public void setPosition(Angle position, PositionControlTypeType positionControlType, ControlSlotType controlSlot) {
            setPosition(position, positionControlType, controlSlot, RadiansPerSecond.of(0));
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
        protected ControlSlotType getControlSlot() {
            return controlSlot;
        }
        protected VelocityControlTypeType getVelocityControlType() {
            return velocityControlType;
        }
        protected PositionControlTypeType getPositionControlType() {
            return positionControlType;
        }
    }

    public static class ArbitraryFeedforward {
        public static enum FeedforwardType {
            DUTY_CYCLE,
            VOLTAGE,
            TORQUE_CURRENT,
        }

        protected FeedforwardType feedforwardType;
        protected double dutyCycle;
        protected double voltage;
        protected double currentAmps;

        public void setDutyCycle(double dutyCycle) {
            feedforwardType = FeedforwardType.DUTY_CYCLE;
            dutyCycle = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        }
        public void setVoltage(Voltage voltage) {
            feedforwardType = FeedforwardType.VOLTAGE;
            this.voltage = voltage.in(Volts);
        }
        public void setTorqueCurrent(Current current) {
            feedforwardType = FeedforwardType.TORQUE_CURRENT;
            currentAmps = current.in(Amps);
        }
    }


    public static class MotorIOConfigRequests<ConfigType extends MotorConfig> {
        protected ConfigType config;
        protected boolean configChanged;

        public void setConfig(ConfigType config) {
            configChanged = true;
            this.config = config;
        }
    }

    /** Applies new config values to motor  */
    public void applyConfigRequests(ConfigRequestsType configRequests);


    public static interface PositionControlType {}

    public static interface VelocityControlType {}

    /** A control slot represents a possible choice of control gains. */
    public static interface ControlSlot {}

    public void setEncoderPosition(Angle position);
}