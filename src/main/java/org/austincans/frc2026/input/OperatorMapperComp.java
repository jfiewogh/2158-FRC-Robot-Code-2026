package org.austincans.frc2026.input;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorMapperComp {
    protected final CommandXboxController controller = new CommandXboxController(0);

    public double getRotationX() {
        return -controller.getRightY();
    }
    public double getRotationY() {
        return -controller.getRightX();
    }
    public Trigger getOuttakeButton() {
        return controller.leftBumper();
    }
	public Trigger getStaticShooterToggleButton() {
		return controller.rightBumper();
	}
	public Trigger getResetTurretAngleButton() {
		return controller.y();
	}
	public Trigger getSetLowestHoodAngleButton() {
		return controller.x();
    }
}
