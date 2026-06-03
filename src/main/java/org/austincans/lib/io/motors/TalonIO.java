package org.austincans.lib.io.motors;

import edu.wpi.first.units.measure.Current;

public abstract class TalonIO extends BaseMotorIO<TalonIO.TalonIOInputs, TalonConfig> {
    protected TalonIO(TalonConfig config) {
        super(config);
    }

    public static class TalonIOInputs extends MotorIO.MotorIOInputs {
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

    protected abstract Current getSupplyCurrent();

    protected abstract Current getTorqueCurrent();

    public static enum TalonControlSlot implements MotorIO.ControlSlot {
        SLOT_0,
        SLOT_1,
        SLOT_2;

        public int getIndex() {
            return this.ordinal();
        }
    }
}
