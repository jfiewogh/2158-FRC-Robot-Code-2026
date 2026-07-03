package org.austincans.lib.io.motors;

import org.austincans.lib.io.ActuatorIO;
import org.austincans.lib.io.ConfigurableIO;

import edu.wpi.first.units.measure.Angle;

public interface MotorIO extends ActuatorIO, ConfigurableIO {
    public void setEncoderPosition(Angle position);
}