package org.austincans.lib.io.motors.outputs.control;

import com.revrobotics.spark.ClosedLoopSlot;

public enum SparkControlSlot implements ControlSlot {
    SLOT_0(ClosedLoopSlot.kSlot0), 
    SLOT_1(ClosedLoopSlot.kSlot1), 
    SLOT_2(ClosedLoopSlot.kSlot2), 
    SLOT_3(ClosedLoopSlot.kSlot3);

    private final ClosedLoopSlot slot;

    private SparkControlSlot(ClosedLoopSlot slot) {
        this.slot = slot;
    }

    public ClosedLoopSlot get() {
        return slot;
    }
    public int getIndex() {
        return ordinal();
    }
}