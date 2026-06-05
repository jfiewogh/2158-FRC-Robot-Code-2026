package org.austincans.lib.io.motors;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public abstract class TalonIO<TalonConfigType extends TalonConfig> extends BaseMotorIO<TalonIO.TalonIOInputs, TalonConfigType, TalonIO.TalonControlSlot, TalonIO.TalonVelocityControlType, TalonIO.TalonPositionControlType> {
    /* Control Requests */
    protected final VoltageOut voltageOut = new VoltageOut(0);
    
    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);
    private final MotionMagicDutyCycle positionMotionMagicDutyCycle = new MotionMagicDutyCycle(0);
    private final MotionMagicVoltage positionMotionMagicVoltage = new MotionMagicVoltage(0);
    private final MotionMagicTorqueCurrentFOC positionMotionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);
    private final MotionMagicExpoDutyCycle positionMotionMagicExpoDutyCycle = new MotionMagicExpoDutyCycle(0);
    private final MotionMagicExpoVoltage positionMotionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
    private final MotionMagicExpoTorqueCurrentFOC positionMotionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
    
    private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
    private final MotionMagicVelocityDutyCycle velocityMotionMagicDutyCycle = new MotionMagicVelocityDutyCycle(0);
    private final MotionMagicVelocityVoltage velocityMotionMagicVoltage = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityTorqueCurrentFOC velocityMotionMagicTorqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(0);

    protected TalonIO(TalonConfigType config) {
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

    protected final ControlRequest getVelocityControlRequest(TalonVelocityControlType velocityControlType, AngularVelocity velocity, TalonControlSlot slot) {
        return switch (velocityControlType) {
            case DUTY_CYCLE -> velocityDutyCycle.withVelocity(velocity).withSlot(slot.get());
            case VOLTAGE -> velocityVoltage.withVelocity(velocity).withSlot(slot.get());
            case TORQUE_CURRENT_FOC -> velocityTorqueCurrentFOC.withVelocity(velocity).withSlot(slot.get());
            case MOTION_MAGIC_DUTY_CYCLE -> velocityMotionMagicDutyCycle.withVelocity(velocity).withSlot(slot.get());
            case MOTION_MAGIC_VOLTAGE -> velocityMotionMagicVoltage.withVelocity(velocity).withSlot(slot.get());
            case MOTION_MAGIC_TORQUE_CURRENT_FOC -> velocityMotionMagicTorqueCurrentFOC.withVelocity(velocity).withSlot(slot.get());
        };
    }

    protected final ControlRequest getPositionControlRequest(TalonPositionControlType positionControlType, Angle position, TalonControlSlot slot) {
        return switch (positionControlType) {
            case DUTY_CYCLE -> positionDutyCycle.withPosition(position).withSlot(slot.get());
            case VOLTAGE -> positionVoltage.withPosition(position).withSlot(slot.get());
            case TORQUE_CURRENT_FOC -> positionTorqueCurrentFOC.withPosition(position).withSlot(slot.get());
            case MOTION_MAGIC_DUTY_CYCLE -> positionMotionMagicDutyCycle.withPosition(position).withSlot(slot.get());
            case MOTION_MAGIC_VOLTAGE -> positionMotionMagicVoltage.withPosition(position).withSlot(slot.get());
            case MOTION_MAGIC_TORQUE_CURRENT_FOC -> positionMotionMagicTorqueCurrentFOC.withPosition(position).withSlot(slot.get());
            case MOTION_MAGIC_EXPO_DUTY_CYCLE -> positionMotionMagicExpoDutyCycle.withPosition(position).withSlot(slot.get());
            case MOTION_MAGIC_EXPO_VOLTAGE -> positionMotionMagicExpoVoltage.withPosition(position).withSlot(slot.get());
            case MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC -> positionMotionMagicExpoTorqueCurrentFOC.withPosition(position).withSlot(slot.get());
        };
    }

    public static enum TalonVelocityControlType implements MotorIO.VelocityControlType {
        DUTY_CYCLE,
        VOLTAGE,
        TORQUE_CURRENT_FOC,
        MOTION_MAGIC_DUTY_CYCLE,
        MOTION_MAGIC_VOLTAGE,
        MOTION_MAGIC_TORQUE_CURRENT_FOC,
    }

    public static enum TalonPositionControlType implements MotorIO.PositionControlType {
        DUTY_CYCLE,
        VOLTAGE,
        TORQUE_CURRENT_FOC,
        MOTION_MAGIC_DUTY_CYCLE,
        MOTION_MAGIC_VOLTAGE,
        MOTION_MAGIC_TORQUE_CURRENT_FOC,
        MOTION_MAGIC_EXPO_DUTY_CYCLE,
        MOTION_MAGIC_EXPO_VOLTAGE,
        MOTION_MAGIC_EXPO_TORQUE_CURRENT_FOC;
    }

    public static enum TalonControlSlot implements MotorIO.ControlSlot {
        SLOT_0,
        SLOT_1,
        SLOT_2;

        public int get() {
            return this.ordinal();
        }
    }
}