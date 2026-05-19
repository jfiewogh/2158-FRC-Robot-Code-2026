package org.austincans.lib.hardware.motors;

import org.austincans.lib.hardware.motors.MotorIO.MotorIOInputs;

import edu.wpi.first.units.measure.Current;

public abstract class TalonIO extends MotorIOReal<TalonIO.TalonIOInputs> {
    protected TalonIO(int deviceId) {
        super(deviceId);
    }

    public class TalonIOInputs extends MotorIOInputs {
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
