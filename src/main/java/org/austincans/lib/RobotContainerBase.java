package org.austincans.lib;

import java.util.function.DoubleSupplier;

public class RobotContainerBase {
    protected DoubleSupplier leftX, leftY, rightX, rightY;

    private boolean axisHasInput(DoubleSupplier axisSupplier) {
        return Math.abs(axisSupplier.getAsDouble()) >= Constants.driveHasInputThreshold;
    }

    public boolean driveHasInput() {
        return axisHasInput(leftX) || axisHasInput(leftY) || axisHasInput(rightX) || axisHasInput(rightY);
    }
}
