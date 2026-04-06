package org.austincans.frc2026.input;

import org.austincans.lib.util.KeyboardHID;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Only used for sim */
public class KeyboardMapper implements DriverMapper {
    private final KeyboardHID keyboard = new KeyboardHID();

    @Override
    public double getTranslationX() {
        return keyboard.leftY();
    }
    @Override
    public double getTranslationY() {
        return keyboard.leftX();
    }
    @Override
    public double getRotationX() {
        return keyboard.rightY();
    }
    @Override
    public double getRotationY() {
        return keyboard.rightX();
    }
    @Override
    public Trigger getShootButton() {
        return keyboard.c();
    }
    @Override
    public Trigger getIntakeToggleButton() {
        return keyboard.v();
    }
    @Override
    public Trigger getTurnModeToggleButton() {
        return keyboard.x();
    }

    public Trigger getDriveAndIntakeButton() {
        return keyboard.m();
    }
    public Trigger getDriveAndScoreButton() {
        return keyboard.comma();
    }
}
