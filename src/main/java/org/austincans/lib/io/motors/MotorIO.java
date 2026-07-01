package org.austincans.lib.io.motors;

import org.austincans.lib.io.ActuatorIO;
import org.austincans.lib.io.ConfigurableIO;
import org.austincans.lib.io.motors.configs.requests.MotorIOConfigRequests;
import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.austincans.lib.io.motors.outputs.control.ControlSlot;
import org.austincans.lib.io.motors.outputs.control.PositionControlType;
import org.austincans.lib.io.motors.outputs.control.VelocityControlType;

import edu.wpi.first.units.measure.Angle;

public interface MotorIO<
    InputsType extends MotorIOInputs, 
    ControlSlotType extends ControlSlot,
    VelocityControlTypeType extends VelocityControlType,
    PositionControlTypeType extends PositionControlType,
    ConfigRequestsType extends MotorIOConfigRequests<ConfigType>,
    ConfigType> 
extends 
    ActuatorIO<InputsType, MotorIOOutputs<ControlSlotType, VelocityControlTypeType, PositionControlTypeType>>, 
    ConfigurableIO<ConfigRequestsType> {

    public void setEncoderPosition(Angle position);
}