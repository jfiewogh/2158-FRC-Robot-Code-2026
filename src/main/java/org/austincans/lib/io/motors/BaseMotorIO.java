package org.austincans.lib.io.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class BaseMotorIO<InputsType extends MotorIO.MotorIOInputs, ConfigType extends MotorConfig<ControlSlotType>, ControlSlotType extends MotorIO.ControlSlot, VelocityControlTypeType extends MotorIO.VelocityControlType, PositionControlTypeType extends MotorIO.PositionControlType> implements MotorIO<InputsType, ConfigType, ControlSlotType, VelocityControlTypeType, PositionControlTypeType> {
    protected final ConfigType config;
    
    public BaseMotorIO(ConfigType config) {
        this.config = config;
    }
    
    /** Updates inputs with newer values from motor */
    public void updateInputs(InputsType inputs) {
        inputs.setPosition(getPosition());
        inputs.setVelocity(getVelocity());
        inputs.setVoltage(getVoltage());
        inputs.setStatorCurrent(getStatorCurrent());
        inputs.setTemperature(getTemperature());
    }

    /** Applies output to motor based on the output mode */
    public final void applyOutputs(MotorIOOutputs<ControlSlotType, VelocityControlTypeType, PositionControlTypeType> outputs) {
        switch (outputs.getMode()) {
            case VOLTAGE -> setVoltage(outputs.getVoltage());
            case VELOCITY -> setVelocity(outputs.getVelocity(), outputs.getControlSlot(), outputs.getVelocityControlType());
            case POSITION -> setPosition(outputs.getPosition(), outputs.getControlSlot(), outputs.getPositionControlType());
        }
    }

    // TODO: 3. Complete this method
    public void applyConfigRequests(ConfigType desiredConfig) {
        if (desiredConfig.slot0Config.equals(config.slot0Config)) {

        }
    }

    /* Position */
    protected abstract Angle getPosition();
    protected abstract void setPosition(Angle position, ControlSlotType slot, PositionControlTypeType positionControlType);

    protected abstract AngularVelocity getVelocity();
    protected abstract void setVelocity(AngularVelocity velocity, ControlSlotType slot, VelocityControlTypeType velocityControlType);

    protected abstract Voltage getVoltage();
    protected abstract void setVoltage(Voltage voltage);

    protected abstract Current getStatorCurrent();

    protected abstract Temperature getTemperature();
}
