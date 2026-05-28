package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.MotorIO.ControlSlot;
import org.austincans.lib.io.motors.MotorIO.MotorIOInputs;

import edu.wpi.first.units.measure.Current;

public abstract class TalonIO extends BaseMotorIO<TalonIO.TalonIOInputs, TalonIO.TalonControlSlot> {
    public static class TalonIOInputs extends MotorIOInputs {
        private Current supplyCurrent;
        private Current torqueCurrent;

        void setSupplyCurrent(Current supplyCurrent) {
            this.supplyCurrent = supplyCurrent;
        }
        void setTorqueCurrent(Current torqueCurrent) {
            this.torqueCurrent = torqueCurrent;
        }

        public Current getSupplyCurrent() {
            return supplyCurrent;
        }
        public Current getTorqueCurrent() {
            return torqueCurrent;
        }
    }

    @Override
    public void updateInputs(TalonIOInputs inputs) {
        super.updateInputs(inputs);
        inputs.setSupplyCurrent(getSupplyCurrent());
        inputs.setTorqueCurrent(getTorqueCurrent());
    }

    abstract Current getSupplyCurrent();

    abstract Current getTorqueCurrent();

    public static enum TalonControlSlot implements ControlSlot {
        SLOT_0(0),
        SLOT_1(1),
        SLOT_2(2);

        private final int slot;

        private TalonControlSlot(int slot) {
            this.slot = slot;
        }
        
        int get() {
            return slot;
        }
    }
}
