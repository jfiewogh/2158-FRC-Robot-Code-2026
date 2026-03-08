package org.austincans.lib.subsystems.drive;

import org.austincans.lib.encoders.CANcoderIO;
import org.austincans.lib.motors.MotorIOTalonFX;
import com.ctre.phoenix6.BaseStatusSignal;

public class ModuleIOTalonFXCANcoder extends ModuleIOBase<MotorIOTalonFX, MotorIOTalonFX, CANcoderIO> {
    public ModuleIOTalonFXCANcoder() {
        
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);
        inputs.driveConnected = BaseStatusSignal.isAllGood(

        );
    }
}
