package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.configs.TalonFXConfig.FeedbackConfig;
import org.austincans.lib.io.motors.configrequests.TalonFXIOConfigRequests;
import org.austincans.lib.io.motors.configs.TalonFXConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO extends TalonIO<TalonFXIOConfigRequests, TalonFXConfig> {
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
    public void applyConfigRequests(TalonFXIOConfigRequests configRequests) {
        if (!configRequests.configChanged()) {
            return;
        }
        configRequests.setConfigChanged(false);

        TalonFXConfig config = configRequests.getConfig();

        int changeCount = getTrueCount(
            configRequests.motorOutputConfigChanged, 
            configRequests.softLimitConfigChanged,
            configRequests.slot0ConfigChanged,
            configRequests.slot1ConfigChanged,
            configRequests.slot2ConfigChanged,
            configRequests.feedbackConfigChanged,
            configRequests.closedLoopGeneralConfigChanged,
            configRequests.motionMagicConfigChanged,
            configRequests.currentLimitsConfigChanged
        );

        /* Multiple Configs */
        if (changeCount > 1) {
            configRequests.motorOutputConfigChanged = false;
            updateMotorOutputConfigs(vendorConfig.MotorOutput, config.motorOutputConfig);

            configRequests.softLimitConfigChanged = false;
            updateSoftLimitSwitchConfigs(vendorConfig.SoftwareLimitSwitch, config.softLimitConfig);

            configRequests.slot0ConfigChanged = false;
            mapSlot0Config(vendorConfig.Slot0, config.slot0Config);

            configRequests.slot1ConfigChanged = false;
            mapSlot1Config(vendorConfig.Slot1, config.slot1Config);

            configRequests.slot2ConfigChanged = false;
            mapSlot2Config(vendorConfig.Slot2, config.slot2Config);

            configRequests.feedbackConfigChanged = false;
            updateFeedbackConfig(vendorConfig.Feedback, config.feedbackConfig);

            configRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(vendorConfig.ClosedLoopGeneral, config.closedLoopGeneralConfig);

            configRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(vendorConfig.MotionMagic, config.motionMagicConfig);

            configRequests.currentLimitsConfigChanged = false;
            updateCurrentLimitsConfig(vendorConfig.CurrentLimits, config.currentLimitsConfig);

            motor.getConfigurator().apply(vendorConfig);
            return;
        }

        /* Individual Configs */
        if (configRequests.motorOutputConfigChanged) {
            configRequests.motorOutputConfigChanged = false;
            updateMotorOutputConfigs(vendorConfig.MotorOutput, config.motorOutputConfig);
            motor.getConfigurator().apply(vendorConfig.MotorOutput);
            return;
        }
        if (configRequests.softLimitConfigChanged) {
            configRequests.softLimitConfigChanged = false;
            updateSoftLimitSwitchConfigs(vendorConfig.SoftwareLimitSwitch, config.softLimitConfig);
            motor.getConfigurator().apply(vendorConfig.SoftwareLimitSwitch);
            return;
        }
        if (configRequests.slot0ConfigChanged) {
            configRequests.slot0ConfigChanged = false;
            mapSlot0Config(vendorConfig.Slot0, config.slot0Config);
            motor.getConfigurator().apply(vendorConfig.Slot0);
            return;
        }
        if (configRequests.slot1ConfigChanged) {
            configRequests.slot1ConfigChanged = false;
            mapSlot1Config(vendorConfig.Slot1, config.slot1Config);
            motor.getConfigurator().apply(vendorConfig.Slot1);
            return;
        }
        if (configRequests.slot2ConfigChanged) {
            configRequests.slot2ConfigChanged = false;
            mapSlot2Config(vendorConfig.Slot2, config.slot2Config);
            motor.getConfigurator().apply(vendorConfig.Slot2);
            return;
        }
        if (configRequests.feedbackConfigChanged) {
            configRequests.feedbackConfigChanged = false;
            updateFeedbackConfig(vendorConfig.Feedback, config.feedbackConfig);
            motor.getConfigurator().apply(vendorConfig.Feedback);
            return;
        }
        if (configRequests.closedLoopGeneralConfigChanged) {
            configRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(vendorConfig.ClosedLoopGeneral, config.closedLoopGeneralConfig);
            motor.getConfigurator().apply(vendorConfig.ClosedLoopGeneral);
            return;
        }
        if (configRequests.motionMagicConfigChanged) {
            configRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(vendorConfig.MotionMagic, config.motionMagicConfig);
            motor.getConfigurator().apply(vendorConfig.MotionMagic);
            return;
        }
        if (configRequests.currentLimitsConfigChanged) {
            configRequests.currentLimitsConfigChanged = false;
            updateCurrentLimitsConfig(vendorConfig.CurrentLimits, config.currentLimitsConfig);
            motor.getConfigurator().apply(vendorConfig.CurrentLimits);
            return;
        }
    }

    /** 
     * See this page for more details on remote sensors
     * https://v6.docs.ctr-electronics.com/en/2025/docs/api-reference/device-specific/talonfx/remote-sensors.html 
     * TODO: Gearbox parameters are temporary. They should become fields of FeedbackConfig.
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
    protected void setPosition(Angle position, TalonPositionControlType positionControlType, TalonControlSlot slot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward) {
        motor.setControl(getPositionControlRequest(positionControlType, position, slot, velocity, arbitraryFeedforward));
    }

    @Override
    protected void setVelocity(AngularVelocity velocity, TalonVelocityControlType velocityControlType, TalonControlSlot slot, ArbitraryFeedforward arbitraryFeedforward) {
        motor.setControl(getVelocityControlRequest(velocityControlType, velocity, slot, arbitraryFeedforward)); 
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