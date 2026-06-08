package org.austincans.lib.io.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.austincans.lib.io.ActuatorIO;
import org.austincans.lib.io.ConfigurableIO;

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
    ConfigType> 
extends 
    ActuatorIO<InputsType, MotorIO.MotorIOOutputs<ControlSlotType, VelocityControlTypeType, PositionControlTypeType>>, 
    ConfigurableIO<ConfigRequestsType> 
{
    /**
     * Values universal for all motors
     * TODO: Current values are universal only for real motors. Eventually, update to be universal for real and sim.  
     */
    public static class MotorIOInputs {
        protected Angle position;
        protected AngularVelocity velocity;
        protected Voltage voltage;
        protected Current statorCurrent;
        protected Temperature temperature;

        public Angle position() {
            return position;
        }
        public AngularVelocity velocity() {
            return velocity;
        }
        public Voltage voltage() {
            return voltage;
        }
        public Current statorCurrent() {
            return statorCurrent;
        }
        public Temperature temperature() {
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
        PositionControlTypeType extends PositionControlType> {

        protected MotorIOOutputMode mode;
        protected Voltage voltage;
        protected AngularVelocity velocity;
        protected Angle position;
        protected ArbitraryFeedforward arbitraryFeedforward;

        protected ControlSlotType controlSlot;
        protected VelocityControlTypeType velocityControlType;
        protected PositionControlTypeType positionControlType;

        public void voltage(Voltage voltage) {
            mode = MotorIOOutputMode.VOLTAGE;
            this.voltage = voltage;
        }

        public void velocity(AngularVelocity velocity, VelocityControlTypeType velocityControlType, ControlSlotType controlSlot, ArbitraryFeedforward arbitraryFeedforward) {
            mode = MotorIOOutputMode.VELOCITY;
            this.velocity = velocity;
            this.arbitraryFeedforward = arbitraryFeedforward;
            this.controlSlot = controlSlot;
            this.velocityControlType = velocityControlType;
        }

        public void position(Angle position, PositionControlTypeType positionControlType, ControlSlotType controlSlot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward) {
            mode = MotorIOOutputMode.POSITION;
            this.position = position;
            this.velocity = velocity;
            this.arbitraryFeedforward = arbitraryFeedforward;
            this.controlSlot = controlSlot;
            this.positionControlType = positionControlType;
        }
        public void position(Angle position, PositionControlTypeType positionControlType, ControlSlotType controlSlot, ArbitraryFeedforward arbitraryFeedforward) {
            position(position, positionControlType, controlSlot, RadiansPerSecond.of(0.0), arbitraryFeedforward);
        }
    }

    /** 
     * If motor requires different feedforward unit than what was set, feedforward will be zero.
     * For example, if the motor requires torque current and you call setVoltage instead of setTorqueCurrent, feedforward will be zero. 
     * TODO: Make a better way of handling this. It should be based on the PositionControlType or VelocityControlType.
     */
    public static class ArbitraryFeedforward {
        public static enum FeedforwardType {
            NONE,
            DUTY_CYCLE,
            VOLTAGE,
            TORQUE_CURRENT,
        }

        protected FeedforwardType feedforwardType;
        protected double dutyCycle;
        protected double volts;
        protected double currentAmps;

        public void dutyCycle(double dutyCycle) {
            feedforwardType = FeedforwardType.DUTY_CYCLE;
            this.dutyCycle = MathUtil.clamp(dutyCycle, -1.0, 1.0);
            volts = 0.0;
            currentAmps = 0.0;
        }
        public void voltage(Voltage voltage) {
            feedforwardType = FeedforwardType.VOLTAGE;
            dutyCycle = 0.0;
            volts = voltage.in(Volts);
            currentAmps = 0.0;
        }
        public void torqueCurrent(Current current) {
            feedforwardType = FeedforwardType.TORQUE_CURRENT;
            dutyCycle = 0.0;
            volts = 0.0;
            currentAmps = current.in(Amps);
        }
        public void none() {
            feedforwardType = FeedforwardType.NONE;
            dutyCycle = 0.0;
            volts = 0.0;
            currentAmps = 0.0;
        }
    }

    public static class MotorIOConfigRequests<ConfigType> {
        protected ConfigType config;
        protected boolean configChanged;

        public void config(ConfigType config) {
            configChanged = true;
            this.config = config;
        }
    }
    
    public static interface PositionControlType {}

    public static interface VelocityControlType {}

    /** A control slot represents a possible choice of control gains. */
    public static interface ControlSlot {}




    public void setEncoderPosition(Angle position);
}