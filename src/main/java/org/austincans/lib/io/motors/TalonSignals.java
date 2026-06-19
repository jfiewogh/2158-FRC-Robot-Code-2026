package org.austincans.lib.io.motors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public record TalonSignals(
    StatusSignal<Angle> position,
    StatusSignal<AngularVelocity> velocity,
    StatusSignal<Voltage> voltage,
    StatusSignal<Current> supplyCurrent,
    StatusSignal<Current> statorCurrent,
    StatusSignal<Current> torqueCurrent,
    StatusSignal<Temperature> temperature,

    StatusSignal<Integer> firmwareVersion,
    StatusSignal<Boolean> hardwareFault,
    StatusSignal<Boolean> bootDuringEnable,
    StatusSignal<Boolean> deviceTemp,
    StatusSignal<Boolean> procTemp
) {
    public BaseStatusSignal[] asArray() {
        return new BaseStatusSignal[] {
            position, velocity, voltage, supplyCurrent, statorCurrent, torqueCurrent, temperature,
            firmwareVersion, hardwareFault, bootDuringEnable, deviceTemp, procTemp
        };
    }
}
