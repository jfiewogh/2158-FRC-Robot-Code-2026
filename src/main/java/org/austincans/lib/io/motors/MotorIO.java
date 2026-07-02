package org.austincans.lib.io.motors;

import org.austincans.lib.io.ActuatorIO;
import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.ConfigurableIO;
import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.austincans.lib.io.motors.outputs.control.ControlSlot;
import org.austincans.lib.io.motors.outputs.control.PositionControlType;
import org.austincans.lib.io.motors.outputs.control.VelocityControlType;

import edu.wpi.first.units.measure.Angle;

/** 
 * TODO: I feel like MotorIO should only care about final objects 
 * such as InputsType, OutputsType, and ConfigRequestsType. 
 * This way, we can have fewer generics.
 * However, to do this, we will have to do downcasting.
 * For example, casting VelocityControlType to SparkVelocityControlType.
 */
public interface MotorIO<
    InputsType extends MotorIOInputs, 
    ControlSlotType extends ControlSlot,
    VelocityControlTypeType extends VelocityControlType,
    PositionControlTypeType extends PositionControlType,
    ConfigRequestsType extends ConfigRequests<ConfigType>,
    ConfigType> 
extends 
    ActuatorIO<InputsType, MotorIOOutputs<ControlSlotType, VelocityControlTypeType, PositionControlTypeType>>, 
    ConfigurableIO<ConfigRequestsType> {

    public void setEncoderPosition(Angle position);
}