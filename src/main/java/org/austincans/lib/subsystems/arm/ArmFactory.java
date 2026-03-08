package org.austincans.lib.subsystems.arm;

import org.austincans.lib.configs.ArmConfig;

public class ArmFactory {
    public ArmBase createArmTalonFXS(ArmConfig armConfig) {
        return new ArmBase(new ArmIOTalonFXSCANcoder(armConfig));
    } 

    public ArmBase createArmSparkFlex(ArmConfig armConfig) {
        return new ArmBase(new ArmIOSparkFlexCANcoder(armConfig));
    } 
}
