package org.austincans.lib.io.motors.outputs.control;

public enum TalonControlSlot implements ControlSlot {
    SLOT_0,
    SLOT_1,
    SLOT_2;

    public int get() {
        return this.ordinal();
    }
}