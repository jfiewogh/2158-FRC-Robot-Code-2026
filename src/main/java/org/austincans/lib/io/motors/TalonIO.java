package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.MotorConfig.ControlConfig;
import org.austincans.lib.io.motors.MotorConfig.SoftLimitConfig;
import org.austincans.lib.io.motors.MotorIO.NeutralMode;

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
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public abstract class TalonIO<
    TalonConfigType extends TalonConfig<FeedbackSensorSourceType>, 
    FeedbackSensorSourceType extends TalonConfig.FeedbackSensorSource
> extends BaseMotorIO<
    TalonIO.TalonIOInputs, 
    TalonIO.TalonControlSlot, 
    TalonIO.TalonVelocityControlType, 
    TalonIO.TalonPositionControlType, 
    TalonIO.TalonIOConfigRequests<TalonConfigType>,
    TalonConfigType> {
    
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

    protected final ControlRequest getPositionControlRequest(TalonPositionControlType positionControlType, Angle position, TalonControlSlot slot, AngularVelocity velocity, Voltage arbitraryFeedforward) {
        return switch (positionControlType) {
            case DUTY_CYCLE -> positionDutyCycle.withPosition(position).withVelocity(velocity).withFeedForward(0).withSlot(slot.get());
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

    protected static int getTrueCount(final boolean... array) {
        int trueCount = 0;
        for (boolean bool : array) {
            if (bool) trueCount++; 
        }
        return trueCount;
    }

    public static class TalonIOConfigRequests<
        TalonConfigType extends TalonConfig<FeedbackSensorSourceType>, 
        FeedbackSensorSourceType extends TalonConfig.FeedbackSensorSource
    > extends MotorIO.MotorIOConfigRequests<TalonConfigType> {
        
        protected boolean softLimitConfigChanged;
        protected boolean motorOutputConfigChanged;
        protected boolean slot0ConfigChanged;
        protected boolean slot1ConfigChanged;
        protected boolean slot2ConfigChanged;
        protected boolean feedbackConfigChanged;
        protected boolean closedLoopGeneralConfigChanged;
        protected boolean motionMagicConfigChanged;
        protected boolean currentLimitsConfigChanged;

        public void setSoftLimitConfig(SoftLimitConfig softLimitConfig) {
            configChanged |= softLimitConfigChanged = !config.mechanismSoftLimitConfig.equals(softLimitConfig);
            config.mechanismSoftLimitConfig = softLimitConfig;
        }

        public void setNeutralMode(NeutralMode neutralMode) {
            configChanged |= motorOutputConfigChanged |= config.neutralMode != neutralMode;
            config.neutralMode = neutralMode;
        }
        public void setInverted(boolean inverted) {
            configChanged |= motorOutputConfigChanged |= config.inverted != inverted;
            config.inverted = inverted;
        }

        public void setSlot0Config(ControlConfig slotConfig) {
            configChanged |= slot0ConfigChanged = !config.slot0Config.equals(slotConfig);
            config.slot0Config = slotConfig;
        }
        public void setSlot1Config(ControlConfig slotConfig) {
            configChanged |= slot1ConfigChanged = !config.slot1Config.equals(slotConfig);
            config.slot1Config = slotConfig;
        }
        public void setSlot2Config(ControlConfig slotConfig) {
            configChanged |= slot2ConfigChanged = !config.slot2Config.equals(slotConfig);
            config.slot2Config = slotConfig;
        }

        public void setFeedbackConfig() {
            configChanged |= feedbackConfigChanged = config.
        }

        public void setPositionContinuousWrap(boolean continuousWrap) {
            configChanged |= closedLoopGeneralConfigChanged |= config.positionContinuousWrap != continuousWrap;
            config.positionContinuousWrap = continuousWrap;
        }

        public void setMotionMagicCruiseVelocity(AngularVelocity velocity) {
            configChanged |= motionMagicConfigChanged |= !config.motionMagicMechanismCruiseVelocity.isEquivalent(velocity);
            config.motionMagicMechanismCruiseVelocity = velocity;
        }
        public void setMotionMagicAcceleration(AngularAcceleration acceleration) {
            configChanged |= motionMagicConfigChanged |= !config.motionMagicMechanismAcceleration.isEquivalent(acceleration);
            config.motionMagicMechanismAcceleration = acceleration;
        }
        public void setMotionMagicJerk(double jerkRotationsPerSecCubed) {
            configChanged |= motionMagicConfigChanged |= config.motionMagicMechanismJerkRotationsPerSecCubed != jerkRotationsPerSecCubed;
            config.motionMagicMechanismJerkRotationsPerSecCubed = jerkRotationsPerSecCubed;
        }

        public void setStatorCurrentLimit(boolean enabled, Current current) {
            configChanged |= currentLimitsConfigChanged |= (config.currentLimitsConfig.statorCurrentLimitEnabled != enabled || !config.currentLimitsConfig.statorCurrentLimit.isEquivalent(current));
            config.currentLimitsConfig.statorCurrentLimitEnabled = enabled;
            config.currentLimitsConfig.statorCurrentLimit = current;
        }
        public void setSupplyCurrentLimit(boolean enabled, Current current) {
            configChanged |= currentLimitsConfigChanged |= (config.currentLimitsConfig.supplyCurrentLimitEnabled != enabled || !config.currentLimitsConfig.supplyCurrentLimit.isEquivalent(current));
            config.currentLimitsConfig.supplyCurrentLimitEnabled = enabled;
            config.currentLimitsConfig.supplyCurrentLimit = current;
        }
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