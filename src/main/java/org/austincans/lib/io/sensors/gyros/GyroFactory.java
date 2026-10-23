package org.austincans.lib.io.sensors.gyros;

import com.ctre.phoenix6.CANBus;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.units.measure.Frequency;

public class GyroFactory {
    public static Pigeon2IO createPigeon2(int deviceId, CANBus canBus, Frequency updateFrequency) {
        return new Pigeon2IO(deviceId, canBus, updateFrequency);
    }

    public static NavXIO createNavX(NavXComType comType, NavXUpdateRate updateRate) {
        return new NavXIO(comType, updateRate);
    }

    public static CanandgyroIO createCanandgyro(int deviceId, Frequency odometryFrequency) {
        return new CanandgyroIO(deviceId, odometryFrequency);
    }
}
