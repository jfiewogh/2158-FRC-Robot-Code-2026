package org.austincans.frc2158.subsystems.led;

import org.austincans.frc2158.commands.TurretCommands;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.flywheel.Flywheel;
import org.austincans.frc2158.subsystems.intake.Intake;
import org.austincans.frc2158.subsystems.led.LedIO.LedIOInputs;
import org.austincans.frc2158.subsystems.led.LedIO.LedState;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase{
    private final LedIO io;
    private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();
    private final Flywheel flywheel;
    private final Drive drive;
    private final Intake intake;

    public Led(LedIO io, Flywheel flywheel, Drive drive, Intake intake){
        this.flywheel = flywheel;
        this.drive = drive;
        this.io = io;
    }

    public void updateState(){
        if (flywheel.readyToShoot()){
            io.setPattern(LedState.READYTOSHOOT);
        }
        else if (flywheel.isShooting()){
            io.setPattern(LedState.SHOOTING);
        }
        else if (drive.isDriving()){
            io.setPattern(LedState.DRIVING);
        }
        else if (intake.distanceSensorActive()){
            io.setPattern(LedState.INTAKED);
        }
        else if (TurretCommands.isShuttling == true){
            io.setPattern(LedState.SHUTTLING);
            TurretCommands.isShuttling = false;
        }
        else{
            io.setPattern(LedState.IDLE);
        }
    }

    @Override
    public void periodic() {
        updateState();
        io.updateInputs(inputs);
    }
}
