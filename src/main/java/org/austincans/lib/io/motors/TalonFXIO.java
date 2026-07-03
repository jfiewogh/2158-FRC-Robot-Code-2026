package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.configs.TalonFXConfig.FeedbackConfig;
import org.austincans.lib.io.motors.configs.requests.TalonFXIOConfigRequests;
import org.austincans.lib.io.motors.outputs.ArbitraryFeedforward;
import org.austincans.lib.io.motors.outputs.control.ControlSlot;
import org.austincans.lib.io.motors.outputs.control.PositionControlType;
import org.austincans.lib.io.motors.outputs.control.TalonControlSlot;
import org.austincans.lib.io.motors.outputs.control.TalonPositionControlType;
import org.austincans.lib.io.motors.outputs.control.TalonVelocityControlType;
import org.austincans.lib.io.motors.outputs.control.VelocityControlType;
import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.motors.configs.TalonFXConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO extends TalonIO {
    private final TalonFX motor;

    private final TalonFXConfiguration vendorConfig = new TalonFXConfiguration();
    
    protected TalonFXIO(int deviceId, CANBus canBus) {
        this(new TalonFX(deviceId, canBus));
    }

    private TalonFXIO(TalonFX motor) {
        super(new TalonSignals(
            motor.getPosition(),
            motor.getVelocity(),
            motor.getMotorVoltage(),
            motor.getSupplyCurrent(),
            motor.getStatorCurrent(),
            motor.getTorqueCurrent(),
            motor.getDeviceTemp(),

            motor.getVersion(),
            motor.getFault_Hardware(),
            motor.getFault_BootDuringEnable(),
            motor.getFault_DeviceTemp(),
            motor.getFault_ProcTemp(), 

            motor.getStickyFault_Hardware(),
            motor.getStickyFault_BootDuringEnable(),
            motor.getStickyFault_Undervoltage(),
            motor.getStickyFault_DeviceTemp(),
            motor.getStickyFault_ProcTemp(),
            motor.getStickyFault_StatorCurrLimit(),
            motor.getStickyFault_SupplyCurrLimit()
        ));

        this.motor = motor;
    }

    @Override
    public void applyConfigRequests(ConfigRequests configRequests) {
        if (!configRequests.configChanged()) {
            return;
        }

        TalonFXIOConfigRequests talonFXConfigRequests = (TalonFXIOConfigRequests) configRequests;
        TalonFXConfig config = (TalonFXConfig) talonFXConfigRequests.getConfig();

        int changeCount = getTrueCount(
            talonFXConfigRequests.motorOutputConfigChanged, 
            talonFXConfigRequests.softLimitConfigChanged,
            talonFXConfigRequests.slot0ConfigChanged,
            talonFXConfigRequests.slot1ConfigChanged,
            talonFXConfigRequests.slot2ConfigChanged,
            talonFXConfigRequests.feedbackConfigChanged,
            talonFXConfigRequests.closedLoopGeneralConfigChanged,
            talonFXConfigRequests.motionMagicConfigChanged,
            talonFXConfigRequests.currentLimitsConfigChanged
        );

        /* Multiple Configs */
        if (changeCount > 1) {
            talonFXConfigRequests.motorOutputConfigChanged = false;
            updateMotorOutputConfigs(vendorConfig.MotorOutput, config.motorOutputConfig);

            talonFXConfigRequests.softLimitConfigChanged = false;
            updateSoftLimitSwitchConfigs(vendorConfig.SoftwareLimitSwitch, config.softLimitConfig);

            talonFXConfigRequests.slot0ConfigChanged = false;
            mapSlot0Config(vendorConfig.Slot0, config.slot0Config);

            talonFXConfigRequests.slot1ConfigChanged = false;
            mapSlot1Config(vendorConfig.Slot1, config.slot1Config);

            talonFXConfigRequests.slot2ConfigChanged = false;
            mapSlot2Config(vendorConfig.Slot2, config.slot2Config);

            talonFXConfigRequests.feedbackConfigChanged = false;
            updateFeedbackConfig(vendorConfig.Feedback, config.feedbackConfig);

            talonFXConfigRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(vendorConfig.ClosedLoopGeneral, config.closedLoopGeneralConfig);

            talonFXConfigRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(vendorConfig.MotionMagic, config.motionMagicConfig);

            talonFXConfigRequests.currentLimitsConfigChanged = false;
            updateCurrentLimitsConfig(vendorConfig.CurrentLimits, config.currentLimitsConfig);

            motor.getConfigurator().apply(vendorConfig);
            return;
        }

        /* Individual Configs */
        if (talonFXConfigRequests.motorOutputConfigChanged) {
            talonFXConfigRequests.motorOutputConfigChanged = false;
            updateMotorOutputConfigs(vendorConfig.MotorOutput, config.motorOutputConfig);
            motor.getConfigurator().apply(vendorConfig.MotorOutput);
            return;
        }
        if (talonFXConfigRequests.softLimitConfigChanged) {
            talonFXConfigRequests.softLimitConfigChanged = false;
            updateSoftLimitSwitchConfigs(vendorConfig.SoftwareLimitSwitch, config.softLimitConfig);
            motor.getConfigurator().apply(vendorConfig.SoftwareLimitSwitch);
            return;
        }
        if (talonFXConfigRequests.slot0ConfigChanged) {
            talonFXConfigRequests.slot0ConfigChanged = false;
            mapSlot0Config(vendorConfig.Slot0, config.slot0Config);
            motor.getConfigurator().apply(vendorConfig.Slot0);
            return;
        }
        if (talonFXConfigRequests.slot1ConfigChanged) {
            talonFXConfigRequests.slot1ConfigChanged = false;
            mapSlot1Config(vendorConfig.Slot1, config.slot1Config);
            motor.getConfigurator().apply(vendorConfig.Slot1);
            return;
        }
        if (talonFXConfigRequests.slot2ConfigChanged) {
            talonFXConfigRequests.slot2ConfigChanged = false;
            mapSlot2Config(vendorConfig.Slot2, config.slot2Config);
            motor.getConfigurator().apply(vendorConfig.Slot2);
            return;
        }
        if (talonFXConfigRequests.feedbackConfigChanged) {
            talonFXConfigRequests.feedbackConfigChanged = false;
            updateFeedbackConfig(vendorConfig.Feedback, config.feedbackConfig);
            motor.getConfigurator().apply(vendorConfig.Feedback);
            return;
        }
        if (talonFXConfigRequests.closedLoopGeneralConfigChanged) {
            talonFXConfigRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(vendorConfig.ClosedLoopGeneral, config.closedLoopGeneralConfig);
            motor.getConfigurator().apply(vendorConfig.ClosedLoopGeneral);
            return;
        }
        if (talonFXConfigRequests.motionMagicConfigChanged) {
            talonFXConfigRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(vendorConfig.MotionMagic, config.motionMagicConfig);
            motor.getConfigurator().apply(vendorConfig.MotionMagic);
            return;
        }
        if (talonFXConfigRequests.currentLimitsConfigChanged) {
            talonFXConfigRequests.currentLimitsConfigChanged = false;
            updateCurrentLimitsConfig(vendorConfig.CurrentLimits, config.currentLimitsConfig);
            motor.getConfigurator().apply(vendorConfig.CurrentLimits);
            return;
        }
    }

    // TODO: Gearbox parameters are temporary. They should become fields of FeedbackConfig.
    /** 
     * See this page for more details on remote sensors
     * https://v6.docs.ctr-electronics.com/en/2025/docs/api-reference/device-specific/talonfx/remote-sensors.html 
     */
    private void updateFeedbackConfig(FeedbackConfigs feedback, FeedbackConfig feedbackConfig) {
        switch (feedbackConfig.feedbackSensorSource) {
            case ROTOR -> {
                feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                feedback.SensorToMechanismRatio = feedbackConfig.sensorToMechanismGearbox.getReduction();
            }
            case REMOTE_CANCODER -> {
                feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                feedback.FeedbackRemoteSensorID = feedbackConfig.feedbackSensorSourceId;
                feedback.RotorToSensorRatio = feedbackConfig.rotorToRemoteSensorGearbox.getReduction();
                feedback.SensorToMechanismRatio = feedbackConfig.sensorToMechanismGearbox.getReduction();
            }
            case FUSED_CANCODER -> {
                feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                feedback.FeedbackRemoteSensorID = feedbackConfig.feedbackSensorSourceId;
                feedback.RotorToSensorRatio = feedbackConfig.rotorToRemoteSensorGearbox.getReduction();
                feedback.SensorToMechanismRatio = feedbackConfig.sensorToMechanismGearbox.getReduction();
            }
        }
    }

    @Override
    protected void setVoltage(Voltage voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    protected void setPosition(Angle position, PositionControlType positionControlType, ControlSlot slot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward) {
        motor.setControl(
            getPositionControlRequest(
                (TalonPositionControlType) positionControlType, 
                position, 
                (TalonControlSlot) slot, 
                velocity, 
                arbitraryFeedforward));
    }

    @Override
    protected void setVelocity(AngularVelocity velocity, VelocityControlType velocityControlType, ControlSlot slot, ArbitraryFeedforward arbitraryFeedforward) {
        motor.setControl(
            getVelocityControlRequest(
                (TalonVelocityControlType) velocityControlType, 
                velocity, 
                (TalonControlSlot) slot, 
                arbitraryFeedforward)); 
    }

    @Override
    public void setEncoderPosition(Angle position) {
        motor.setPosition(position);
    }

    @Override
    public void clearStickyFaults() {
        motor.clearStickyFaults();
    }
}