package org.austincans.frc2026.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverMapper {
    public double getTranslationX();
    public double getTranslationY();
    public double getRotationX();
    public double getRotationY();

    public Trigger getShootButton();
    public Trigger getRollerToggleButton();
    public Trigger getTurnModeToggleButton();
}
