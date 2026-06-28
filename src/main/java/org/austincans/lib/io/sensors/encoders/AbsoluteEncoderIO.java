package org.austincans.lib.io.sensors.encoders;

import org.austincans.lib.io.DeviceIO;
import org.austincans.lib.io.sensors.encoders.configrequests.AbsoluteEncoderIOConfigRequests;
import org.austincans.lib.io.sensors.encoders.inputs.AbsoluteEncoderIOInputs;
import org.austincans.lib.io.ConfigurableIO;

public interface AbsoluteEncoderIO<
    InputsType extends AbsoluteEncoderIOInputs,
    ConfigRequestsType extends AbsoluteEncoderIOConfigRequests<ConfigType>,
    ConfigType>
extends DeviceIO<InputsType>, ConfigurableIO<ConfigRequestsType> {}
