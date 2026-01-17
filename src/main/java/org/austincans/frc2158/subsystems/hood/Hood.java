package org.austincans.frc2158.subsystems.hood;

import org.austincans.frc2158.subsystems.hood.HoodIO.hoodIOInputs;
import org.littletonrobotics.junction.Logger;

import org.austincans.frc2158.math.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase{
    private HoodIO io;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private Angle angle;

    Command runHoodMotorCommand(Angle angle){
        return Commands.run((() -> io.setHoodAngle(angle)));
    }

    public void setAngle(Angle angle){
        io.setHoodAngle(angle);
    }

    public Angle getAngle(){
        io.getAngle();
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

}
