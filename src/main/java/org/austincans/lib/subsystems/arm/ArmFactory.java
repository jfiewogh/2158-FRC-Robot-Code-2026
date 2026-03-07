package org.austincans.lib.subsystems.arm;

public class ArmFactory {
    public ArmBase createArmTalonFXS(String name, ArmConfig armConfig) {
        return new ArmBase(name, new ArmIOTalonFXS(armConfig));
    } 

    public ArmBase createArmSparkFlex(String name, ArmConfig armConfig) {
        return new ArmBase(name, new ArmIOSparkFlex(armConfig));
    } 
}
