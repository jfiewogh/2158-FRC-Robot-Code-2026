package org.austincans.lib.motors;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO<InputsType extends MotorIOInputs> {
    /**
     * It is plausible not all motors have a neutral/idle mode.
     * If true, then MotorIORequests should not have a NeutralMode.
     * You have a class that extends MotorIORequests for motors that have neutral/idle mode.
     */
    public enum NeutralMode {
        BRAKE,
        COAST
    }

    public enum MotorIOOutputMode {
        VOLTAGE,
        VELOCITY,
        POSITION
    }

    public class MotorIOOutputs {
        public MotorIOOutputMode mode;
        public AngleUnit positionMode;
        public AngularVelocityUnit velocityMode;
        public Voltage voltage;
        public AngularVelocity velocity;
        public Angle position;
        public NeutralMode neutralMode;

        public void setVoltage(Voltage voltage) {
            mode = MotorIOOutputMode.VOLTAGE;
            this.voltage = voltage;
        }
        public void setVelocity(AngularVelocity velocity) {
            mode = MotorIOOutputMode.VELOCITY;
            this.velocity = velocity;
        }
        public void setPosition(Angle position) {
            mode = MotorIOOutputMode.POSITION;
            this.position = position;
        }
        public void setNeutralMode(NeutralMode neutralMode) {
            this.neutralMode = neutralMode;
        }
        public double getVelocity(AngularVelocityUnit unit){
            return this.velocity.in(unit);
        }
        public double getPosition(AngleUnit unit){
            return this.position.in(unit);
        }
    }

    public void updateInputs(InputsType inputs);

    public void applyOutputs(MotorIOOutputs outputs);
}
