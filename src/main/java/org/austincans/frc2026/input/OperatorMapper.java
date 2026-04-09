package org.austincans.frc2026.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorMapper {
    public double getRotationX();
    public double getRotationY();

    public Trigger getOuttakeButton();
	public Trigger getStaticShooterToggleButton();
	public Trigger getResetTurretAngleButton();
	public Trigger getSetLowestHoodAngleButton();
}
