package org.austincans.lib.io.motors.outputs;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.austincans.lib.io.Outputs;
import org.austincans.lib.io.motors.outputs.control.ControlSlot;
import org.austincans.lib.io.motors.outputs.control.PositionControlType;
import org.austincans.lib.io.motors.outputs.control.VelocityControlType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class MotorIOOutputs implements Outputs {
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

    private ControlSlot controlSlot;
    private VelocityControlType velocityControlType;
    private PositionControlType positionControlType;

    public void setVoltage(Voltage voltage) {
        mode = MotorIOOutputMode.VOLTAGE;
        this.voltage = voltage;
    }
    public void setVelocity(AngularVelocity velocity, VelocityControlType velocityControlType, ControlSlot controlSlot, ArbitraryFeedforward arbitraryFeedforward) {
        mode = MotorIOOutputMode.VELOCITY;
        this.velocity = velocity;
        this.arbitraryFeedforward = arbitraryFeedforward;
        this.controlSlot = controlSlot;
        this.velocityControlType = velocityControlType;
    }

    public void setPosition(Angle position, PositionControlType positionControlType, ControlSlot controlSlot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward) {
        mode = MotorIOOutputMode.POSITION;
        this.position = position;
        this.velocity = velocity;
        this.arbitraryFeedforward = arbitraryFeedforward;
        this.controlSlot = controlSlot;
        this.positionControlType = positionControlType;
    }
    public void setPosition(Angle position, PositionControlType positionControlType, ControlSlot controlSlot, ArbitraryFeedforward arbitraryFeedforward) {
        setPosition(position, positionControlType, controlSlot, RadiansPerSecond.of(0.0), arbitraryFeedforward);
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
    public ControlSlot getControlSlot() {
        return controlSlot;
    }
    public VelocityControlType getVelocityControlType() {
        return velocityControlType;
    }
    public PositionControlType getPositionControlType() {
        return positionControlType;
    }
}