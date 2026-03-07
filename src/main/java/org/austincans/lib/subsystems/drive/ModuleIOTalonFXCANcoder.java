package org.austincans.lib.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;

public class ModuleIOTalonFXCANcoder extends ModuleIOBase {
    public ModuleIOTalonFXCANcoder

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);
        inputs.driveConnected = BaseStatusSignal.isAllGood(

        );
    }
}
