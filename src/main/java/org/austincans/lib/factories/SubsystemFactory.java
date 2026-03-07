package org.austincans.lib.factories;

import org.austincans.lib.subsystems.arm.ArmBase;

public class SubsystemFactory {
    public ArmBase createArmBase() {
        return new ArmBase();
    }
}
