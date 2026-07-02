package org.austincans.lib.io.sensors.encoders;

import org.austincans.lib.io.DeviceIO;
import org.austincans.lib.io.sensors.encoders.inputs.AbsoluteEncoderIOInputs;
import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.ConfigurableIO;

public interface AbsoluteEncoderIO<
    InputsType extends AbsoluteEncoderIOInputs,
    ConfigRequestsType extends ConfigRequests<ConfigType>,
    ConfigType>
extends DeviceIO<InputsType>, ConfigurableIO<ConfigRequestsType> {}
