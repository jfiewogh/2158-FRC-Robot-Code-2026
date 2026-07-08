package org.austincans.lib.io.motors.outputs;

import org.austincans.lib.io.Outputs;
import org.austincans.lib.io.motors.outputs.control.PositionControlInputs;
import org.austincans.lib.io.motors.outputs.control.VelocityControlInputs;
import edu.wpi.first.units.measure.Voltage;

public class MotorIOOutputs implements Outputs {
    public static enum MotorIOOutputMode {
        VOLTAGE,
        VELOCITY,
        POSITION
    }

    private MotorIOOutputMode mode;
    private Voltage voltage;
    private VelocityControlInputs velocityControlInputs;
    private PositionControlInputs positionControlInputs;

    public void setVoltage(Voltage voltage) {
        mode = MotorIOOutputMode.VOLTAGE;
        this.voltage = voltage;
    }
    public void setVelocity(VelocityControlInputs velocityControlInputs) {
        mode = MotorIOOutputMode.VELOCITY;
        this.velocityControlInputs = velocityControlInputs;
    }
    public void setPosition(PositionControlInputs positionControlInputs) {
        mode = MotorIOOutputMode.POSITION;
        this.positionControlInputs = positionControlInputs;
    }

    public MotorIOOutputMode getMode() {
        return mode;
    }
    public Voltage getVoltage() {
        return voltage;
    }
    public VelocityControlInputs getVelocityControlInputs() {
        return velocityControlInputs;
    }
    public PositionControlInputs getPositionControlInputs() {
        return positionControlInputs;
    }
}