package org.austincans.lib.io.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.austincans.lib.io.motors.MotorConfig.ControlConfig;
import org.austincans.lib.io.motors.MotorConfig.SoftLimitConfig;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO extends TalonIO {
    private final TalonFX motor;

    /** 
     * TODO: Config should decide which to used 
     * TODO: Handle PID values for multiple controls (ex. config should include slot such as Slot0, Slot1, etc.)
     */
    private final VoltageOut voltageOut = new VoltageOut(0);
    private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);
    private final MotionMagicVelocityVoltage velocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(1);

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;

    TalonFXIO(TalonFXConfig config) {
        super(config);

        motor = new TalonFX(config.deviceId, config.canBus);

        TalonFXConfiguration vendorConfig = new TalonFXConfiguration();

        /** 
         * See this page for more details on configuration
         * https://v6.docs.ctr-electronics.com/en/2025/docs/api-reference/device-specific/talonfx/remote-sensors.html 
         */
        switch (config.feedbackSensorSource) {
            case ROTOR -> {
                vendorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                vendorConfig.Feedback.SensorToMechanismRatio = config.sensorToMechanismGearbox.getReduction();
            }
            case REMOTE_CANCODER -> {
                vendorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                vendorConfig.Feedback.FeedbackRemoteSensorID = config.feedbackSensorSourceId;
                vendorConfig.Feedback.RotorToSensorRatio = config.rotorToRemoteSensor.getReduction();
            }
            case FUSED_CANCODER -> {
                vendorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                vendorConfig.Feedback.FeedbackRemoteSensorID = config.feedbackSensorSourceId;
                vendorConfig.Feedback.RotorToSensorRatio = config.rotorToRemoteSensor.getReduction();
            }
        }

        vendorConfig.MotionMagic.MotionMagicCruiseVelocity = config.maxSpeed.in(RotationsPerSecond);
        vendorConfig.MotionMagic.MotionMagicAcceleration = config.maxAcceleration.in(RotationsPerSecondPerSecond);

        /** Figure out a better way to do this if you can. It is long */
        vendorConfig.Slot0.kP = config.slot0Config.pidConfig.kP;
        vendorConfig.Slot0.kI = config.slot0Config.pidConfig.kI;
        vendorConfig.Slot0.kD = config.slot0Config.pidConfig.kD;
        vendorConfig.Slot0.kS = config.slot0Config.ffConfig.kS;
        vendorConfig.Slot0.kG = config.slot0Config.ffConfig.kG;
        vendorConfig.Slot0.kV = config.slot0Config.ffConfig.kV;
        vendorConfig.Slot0.kA = config.slot0Config.ffConfig.kA;

        vendorConfig.Slot1.kP = config.slot1Config.pidConfig.kP;
        vendorConfig.Slot1.kI = config.slot1Config.pidConfig.kI;
        vendorConfig.Slot1.kD = config.slot1Config.pidConfig.kD;
        vendorConfig.Slot1.kS = config.slot1Config.ffConfig.kS;
        vendorConfig.Slot1.kG = config.slot1Config.ffConfig.kG;
        vendorConfig.Slot1.kV = config.slot1Config.ffConfig.kV;
        vendorConfig.Slot1.kA = config.slot1Config.ffConfig.kA;

        vendorConfig.Slot2.kP = config.slot2Config.pidConfig.kP;
        vendorConfig.Slot2.kI = config.slot2Config.pidConfig.kI;
        vendorConfig.Slot2.kD = config.slot2Config.pidConfig.kD;
        vendorConfig.Slot2.kS = config.slot2Config.ffConfig.kS;
        vendorConfig.Slot2.kG = config.slot2Config.ffConfig.kG;
        vendorConfig.Slot2.kV = config.slot2Config.ffConfig.kV;
        vendorConfig.Slot2.kA = config.slot2Config.ffConfig.kA;
        
        SoftLimitConfig softLimitConfig = config.softLimitConfig;
        vendorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = softLimitConfig.forwardSoftLimitEnabled;
        vendorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softLimitConfig.forwardSoftLimit.in(Rotations);
        vendorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = softLimitConfig.reverseSoftLimitEnabled;
        vendorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softLimitConfig.reverseSoftLimit.in(Rotations);

        vendorConfig.MotorOutput.NeutralMode = config.neutralMode == NeutralMode.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        
        vendorConfig.MotorOutput.Inverted = config.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        vendorConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit.in(Amps);

        motor.getConfigurator().apply(vendorConfig);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temperature = motor.getDeviceTemp();
    }

    /** TODO: Figure out a better way to do this if you can. It seems too long */
    @Override
    public void updateGains(ControlConfig controlConfig, ControlSlot slot) {
        switch (slot.getIndex()) {
            case 0 -> {
                Slot0Configs slot0 = new Slot0Configs();
                slot0.kP = controlConfig.pidConfig.kP;
                slot0.kI = controlConfig.pidConfig.kI;
                slot0.kD = controlConfig.pidConfig.kD;
                slot0.kS = controlConfig.ffConfig.kS;
                slot0.kG = controlConfig.ffConfig.kG;
                slot0.kV = controlConfig.ffConfig.kV;
                slot0.kA = controlConfig.ffConfig.kA;
                motor.getConfigurator().apply(slot0);
                return;
            }
            case 1 -> {
                Slot1Configs slot1 = new Slot1Configs();
                slot1.kP = controlConfig.pidConfig.kP;
                slot1.kI = controlConfig.pidConfig.kI;
                slot1.kD = controlConfig.pidConfig.kD;
                slot1.kS = controlConfig.ffConfig.kS;
                slot1.kG = controlConfig.ffConfig.kG;
                slot1.kV = controlConfig.ffConfig.kV;
                slot1.kA = controlConfig.ffConfig.kA;
                motor.getConfigurator().apply(slot1);
                return;
            }
            case 2 -> {
                Slot2Configs slot2 = new Slot2Configs();
                slot2.kP = controlConfig.pidConfig.kP;
                slot2.kI = controlConfig.pidConfig.kI;
                slot2.kD = controlConfig.pidConfig.kD;
                slot2.kS = controlConfig.ffConfig.kS;
                slot2.kG = controlConfig.ffConfig.kG;
                slot2.kV = controlConfig.ffConfig.kV;
                slot2.kA = controlConfig.ffConfig.kA;
                motor.getConfigurator().apply(slot2);
                return;
            }
        }
    }

    @Override
    protected void setVoltage(Voltage voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    protected void setVelocity(AngularVelocity velocity, ControlSlot slot) {
        motor.setControl(velocityVoltage.withVelocity(velocity).withSlot(slot.getIndex())); 
    }

    @Override
    protected void setPosition(Angle position, ControlSlot slot) {
        motor.setControl(positionVoltage.withPosition(position).withSlot(slot.getIndex()));
    }

    @Override
    protected Angle getPosition() {
        return position.getValue();
    }

    @Override
    protected AngularVelocity getVelocity() {
        return velocity.getValue();
    }

    @Override
    protected Voltage getVoltage() {
        return voltage.getValue();
    }

    @Override
    protected Current getStatorCurrent() {
        return statorCurrent.getValue();
    }

    @Override
    protected Current getSupplyCurrent() {
        return supplyCurrent.getValue();
    }

    @Override
    protected Current getTorqueCurrent() {
        return torqueCurrent.getValue();
    }

    @Override
    protected Temperature getTemperature() {
        return temperature.getValue();
    }

    @Override
    public void initializeOutputs(MotorIOOutputs outputs) {
        outputs.setControlSlot(config.initialSlot);
    }
}
