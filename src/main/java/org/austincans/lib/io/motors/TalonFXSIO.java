package org.austincans.lib.io.motors;

import java.util.ArrayList;

import org.austincans.lib.io.motors.TalonFXSConfig.ExternalFeedbackConfig;
import org.austincans.lib.io.motors.TalonFXSConfig.MotorArrangement;
import org.austincans.lib.io.motors.configrequests.TalonFXSIOConfigRequests;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXSIO extends TalonIO<TalonFXSIOConfigRequests, TalonFXSConfig> {
    private final TalonFXS motor;

    private final TalonFXSConfiguration vendorConfig = new TalonFXSConfiguration();

    protected TalonFXSIO(int deviceId, CANBus canBus) {
        motor = new TalonFXS(deviceId, canBus);

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
    }

    @Override
    public void applyConfigRequests(TalonFXSIOConfigRequests configRequests) {
        if (!configRequests.configChanged()) {
            return;
        }
        configRequests.setConfigChanged(false);

        TalonFXSConfig config = configRequests.getConfig();

        int changeCount = getTrueCount(
            configRequests.motorOutputConfigChanged, 
            configRequests.softLimitConfigChanged,
            configRequests.slot0ConfigChanged,
            configRequests.slot1ConfigChanged,
            configRequests.slot2ConfigChanged,
            configRequests.externalFeedbackConfigChanged,
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

            configRequests.externalFeedbackConfigChanged = false;
            updateExternalFeedbackConfig(vendorConfig.ExternalFeedback, config.externalFeedbackConfig);

            configRequests.closedLoopGeneralConfigChanged = false;
            updateClosedLoopGeneralConfig(vendorConfig.ClosedLoopGeneral, config.closedLoopGeneralConfig);

            configRequests.motionMagicConfigChanged = false;
            updateMotionMagicConfig(vendorConfig.MotionMagic, config.motionMagicConfig);

            configRequests.currentLimitsConfigChanged = false;
            updateCurrentLimitsConfig(vendorConfig.CurrentLimits, config.currentLimitsConfig);

            configRequests.commutationConfigChanged = false;
            updateCommutationConfig(vendorConfig.Commutation, config.motorArrangement);

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
        if (configRequests.externalFeedbackConfigChanged) {
            configRequests.externalFeedbackConfigChanged = false;
            updateExternalFeedbackConfig(vendorConfig.ExternalFeedback, config.externalFeedbackConfig);
            motor.getConfigurator().apply(vendorConfig.ExternalFeedback);
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

    /** https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/talonfx_swerve/src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFXS.java */
    private void updateCommutationConfig(CommutationConfigs commutation, MotorArrangement motorArrangement) {
        commutation.MotorArrangement = switch (motorArrangement) {
            case MINION_JST -> MotorArrangementValue.Minion_JST;
            case NEO_JST -> MotorArrangementValue.NEO_JST;
            case VORTEX_JST -> MotorArrangementValue.VORTEX_JST;
            case NEO550_JST -> MotorArrangementValue.NEO550_JST;
            case BRUSHED_AB, BRUSHED_AC, BRUSHED_BC -> MotorArrangementValue.Brushed_DC;
            case DISABLED -> MotorArrangementValue.Disabled;
        };
        commutation.BrushedMotorWiring = switch (motorArrangement) {
            case BRUSHED_AC -> BrushedMotorWiringValue.Leads_A_and_C;
            case BRUSHED_BC ->BrushedMotorWiringValue.Leads_B_and_C;
            default -> BrushedMotorWiringValue.Leads_A_and_B;
        };     
    }

    private void updateExternalFeedbackConfig(ExternalFeedbackConfigs externalFeedback, ExternalFeedbackConfig externalFeedbackConfig) {
        switch (externalFeedbackConfig.externalFeedbackSensorSource) {
            /** Commutation is what is driven/controlled by the motor controller (TalonFXS) */
            case COMMUTATION -> {
                externalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
                externalFeedback.FeedbackRemoteSensorID = externalFeedbackConfig.externalFeedbackSensorSourceId;
                externalFeedback.RotorToSensorRatio = externalFeedbackConfig.rotorToRemoteSensorGearbox.getReduction();
                externalFeedback.SensorToMechanismRatio = externalFeedbackConfig.sensorToMechanismGearbox.getReduction();
            }
            case REMOTE_CANCODER -> {
                externalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
                externalFeedback.FeedbackRemoteSensorID = externalFeedbackConfig.externalFeedbackSensorSourceId;
                externalFeedback.RotorToSensorRatio = externalFeedbackConfig.rotorToRemoteSensorGearbox.getReduction();
                externalFeedback.SensorToMechanismRatio = externalFeedbackConfig.sensorToMechanismGearbox.getReduction();
            }
            case FUSED_CANCODER -> {
                externalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
                externalFeedback.FeedbackRemoteSensorID = externalFeedbackConfig.externalFeedbackSensorSourceId;
                externalFeedback.RotorToSensorRatio = externalFeedbackConfig.rotorToRemoteSensorGearbox.getReduction();
                externalFeedback.SensorToMechanismRatio = externalFeedbackConfig.sensorToMechanismGearbox.getReduction();
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