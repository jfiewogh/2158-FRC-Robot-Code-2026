package org.austincans.frc2026.input;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorMapperComp implements OperatorMapper {
    protected final CommandXboxController controller = new CommandXboxController(0);

    @Override
    public double getRotationX() {
        return -controller.getRightY();
    }
    @Override
    public double getRotationY() {
        return -controller.getRightX();
    }
    @Override
    public Trigger getOuttakeButton() {
        return controller.leftBumper();
    }
    @Override
	public Trigger getStaticShooterToggleButton() {
		return controller.rightBumper();
	}
    @Override
	public Trigger getResetTurretAngleButton() {
		return controller.y();
	}
    @Override
	public Trigger getSetLowestHoodAngleButton() {
		return controller.x();
    }
}
