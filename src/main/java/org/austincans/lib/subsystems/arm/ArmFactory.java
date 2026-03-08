package org.austincans.lib.subsystems.arm;

public class ArmFactory {
    public ArmBase createArmTalonFXS(ArmConfig armConfig) {
        return new ArmBase(new ArmIOTalonFXSCANcoder(armConfig));
    } 

    public ArmBase createArmSparkFlex(ArmConfig armConfig) {
        return new ArmBase(new ArmIOSparkFlexCANcoder(armConfig));
    } 
}
