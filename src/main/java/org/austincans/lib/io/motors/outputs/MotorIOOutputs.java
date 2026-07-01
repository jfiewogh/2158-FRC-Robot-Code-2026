package org.austincans.lib.io.motors.outputs;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.austincans.lib.io.motors.outputs.control.ControlSlot;
import org.austincans.lib.io.motors.outputs.control.PositionControlType;
import org.austincans.lib.io.motors.outputs.control.VelocityControlType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class MotorIOOutputs<
    ControlSlotType extends ControlSlot, 
    VelocityControlTypeType extends VelocityControlType, 
    PositionControlTypeType extends PositionControlType> {

    public static enum MotorIOOutputMode {
        VOLTAGE,
        VELOCITY,
        POSITION
    }

    private MotorIOOutputMode mode;
    private Voltage voltage;
    private AngularVelocity velocity;
    private Angle position;
    private ArbitraryFeedforward arbitraryFeedforward;

    private ControlSlotType controlSlot;
    private VelocityControlTypeType velocityControlType;
    private PositionControlTypeType positionControlType;

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

    public MotorIOOutputMode getMode() {
        return mode;
    }
    public Voltage getVoltage() {
        return voltage;
    }
    public AngularVelocity getVelocity() {
        return velocity;
    }
    public Angle getPosition() {
        return position;
    }
    public ArbitraryFeedforward getArbitraryFeedforward() {
        return arbitraryFeedforward;
    }
    public ControlSlotType getControlSlot() {
        return controlSlot;
    }
    public VelocityControlTypeType getVelocityControlType() {
        return velocityControlType;
    }
    public PositionControlTypeType getPositionControlType() {
        return positionControlType;
    }
}