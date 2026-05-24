package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.MotorIO.MotorIOInputs;

import edu.wpi.first.units.measure.Current;

public abstract class TalonIO extends BaseMotorIO<TalonIO.TalonIOInputs> {
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
}
