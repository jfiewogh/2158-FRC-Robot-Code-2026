package org.austincans.frc2158.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs{
        double position = 0;
        double voltage = 0;
        double current = 0;
        double velocity = 0;
        double temperatureC = 0;
    }

    public default void setVoltage(double voltage){}

    public default void setPosition(double heightMeters){}

    public default void setVelocity(){}

    public default void updateInputs(ElevatorIOInputs inputs){}
}
