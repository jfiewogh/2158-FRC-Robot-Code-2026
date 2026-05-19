package org.austincans.lib.motors;

import edu.wpi.first.units.measure.Current;

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
