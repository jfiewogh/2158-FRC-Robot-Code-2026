package org.austincans.frc2026.input;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverMapperComp implements DriverMapper {
    private final CommandXboxController controller = new CommandXboxController(MapperConstants.driverPort);

    @Override
    public double getTranslationX() {
        return -controller.getLeftY();
    }
    @Override
    public double getTranslationY() {
        return -controller.getLeftX();
    }
    @Override
    public double getRotationX() {
        return -controller.getRightY();
    }
    @Override
    public double getRotationY() {
        return -controller.getRightX();
    }
    @Override
    public Trigger getShootButton() {
        return controller.rightBumper();
    }
    @Override
    public Trigger getIntakeToggleButton() {
        return controller.leftBumper();
    }
    @Override
    public Trigger getTurnModeToggleButton() {
        return controller.b();
    }

    public Trigger getDeployButton() {
        return controller.leftTrigger();
    }
    public Trigger getRetractButton() {
        return controller.rightTrigger();
    }
    public Trigger getResetPoseButton() {
        return controller.back();
    }
    public Trigger getStopWithXButton() {
        return controller.x();
    }
}