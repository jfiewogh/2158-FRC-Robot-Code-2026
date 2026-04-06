package org.austincans.frc2026.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorMapperTest extends OperatorMapperComp {
    public Trigger getPointToHubButton() {
        return controller.povDown();
    }

    public Trigger getIncrementHoodAngleButton() {
        return controller.b();
    }
    public Trigger getDecrementHoodAngleButton() {
        return controller.a();
    }
    public Trigger getRunIndexerAndFlywheelButton() {
        return controller.povUp();
    }
}