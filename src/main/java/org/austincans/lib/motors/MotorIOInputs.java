package org.austincans.lib.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** 
 * Includes values that are common for all motors. 
 * Values that are not common should be separated into other classes. 
 * TODO: Currently, the values in here are for a real motor. Eventually, update them to be common for real and sim.  
 */
public class MotorIOInputs {
    private boolean connected;
    private Angle position;
    private AngularVelocity velocity;
    private Voltage voltage;
    private Current statorCurrent;
    private Temperature temperature;

    void setConnected(boolean connected) {
        this.connected = connected;
    }
    void setPosition(Angle position) {
        this.position = position;
    }
    void setVelocity(AngularVelocity velocity) {
        this.velocity = velocity;
    }
    void setVoltage(Voltage voltage) {
        this.voltage = voltage;
    }
    void setStatorCurrent(Current statorCurrent) {
        this.statorCurrent = statorCurrent;
    }
    void setTemperature(Temperature temperature) {
        this.temperature = temperature;
    }

    public boolean isConnected() {
        return connected;
    }
    public Angle getPosition() {
        return position;
    }
    public AngularVelocity getVelocity() {
        return velocity;
    }
    public Voltage getVoltage() {
        return voltage;
    }
    public Current getStatorCurrent() {
        return statorCurrent;
    }
    public Temperature getTemperature() {
        return temperature;
    }
}    