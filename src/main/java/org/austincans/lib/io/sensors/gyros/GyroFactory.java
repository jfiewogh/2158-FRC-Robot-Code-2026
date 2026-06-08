package org.austincans.lib.io.sensors.gyros;

import com.ctre.phoenix6.CANBus;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.units.measure.Frequency;

public class GyroFactory {
    public static GyroIOPigeon2 createPigeon2(int deviceId, CANBus canBus, Frequency updateFrequency) {
        return new GyroIOPigeon2(deviceId, canBus, updateFrequency);
    }

    public static GyroIONavX createNavX(NavXComType comType, NavXUpdateRate updateRate) {
        return new GyroIONavX(comType, updateRate);
    }

    public static GyroIORedux createCanandgyro(int deviceId, Frequency odometryFrequency) {
        return new GyroIORedux(deviceId, odometryFrequency);
    }
}
