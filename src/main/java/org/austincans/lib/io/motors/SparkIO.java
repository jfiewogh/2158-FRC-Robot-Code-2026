package org.austincans.lib.io.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.austincans.lib.io.motors.configs.SparkConfig.ClosedLoopConfig.SlotConfig;
import org.austincans.lib.io.motors.configs.SparkConfig.SignalsConfig;
import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.Inputs;
import org.austincans.lib.io.motors.configs.ControlConfig;
import org.austincans.lib.io.motors.configs.NeutralMode;
import org.austincans.lib.io.motors.configs.SparkConfig;
import org.austincans.lib.io.motors.inputs.SparkIOInputs;
import org.austincans.lib.io.motors.outputs.ArbitraryFeedforward;
import org.austincans.lib.io.motors.outputs.control.ControlSlot;
import org.austincans.lib.io.motors.outputs.control.PositionControlType;
import org.austincans.lib.io.motors.outputs.control.SparkControlSlot;
import org.austincans.lib.io.motors.outputs.control.SparkPositionControlType;
import org.austincans.lib.io.motors.outputs.control.SparkVelocityControlType;
import org.austincans.lib.io.motors.outputs.control.VelocityControlType;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * To configure while deployed, use kNoResetSafeParameters and kNoPersistParameters 
 * so that you can configure only specific values while keeping existing ones.
 */
public class SparkIO extends AbstractMotorIO {
    protected final SparkBase motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    private final SparkBaseConfig vendorConfig;
    private boolean initialConfigApplied = false;

    private final ControlConfig[] slotControlConfigs = new ControlConfig[4];

    protected SparkIO(SparkBase motor, SparkBaseConfig vendorConfig) {
        this.vendorConfig = vendorConfig;

        this.motor = motor;
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(Inputs inputs) {
        SparkIOInputs sparkInputs = (SparkIOInputs) inputs;

        super.updateInputs(sparkInputs);

        Faults activeFaults = motor.getFaults();
        sparkInputs.activeFaultMotorType = activeFaults.motorType;
        sparkInputs.activeFaultCan = activeFaults.can;
        sparkInputs.activeFaultTemperature = activeFaults.temperature;
        sparkInputs.activeFaultGateDriver = activeFaults.gateDriver;
        sparkInputs.activeFaultEscEeprom = activeFaults.escEeprom;
        sparkInputs.activeFaultFirmware = activeFaults.firmware;

        Faults stickyFaults = motor.getStickyFaults();
        sparkInputs.stickyFaultMotorType = stickyFaults.motorType;
        sparkInputs.stickyFaultCan = stickyFaults.can;
        sparkInputs.stickyFaultTemperature = stickyFaults.temperature;
        sparkInputs.stickyFaultGateDriver = stickyFaults.gateDriver;
        sparkInputs.stickyFaultEscEeprom = stickyFaults.escEeprom;
        sparkInputs.stickyFaultFirmware = stickyFaults.firmware;
        sparkInputs.stickyFaultOther = stickyFaults.other;
    }

    @Override
    protected void setVoltage(Voltage voltage) {
        controller.setSetpoint(voltage.in(Volts), ControlType.kVoltage);
    }

    /** 
     * Helpful page that explains feedforward control:
     * https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control 
     */
    @Override
    protected void setVelocity(AngularVelocity velocity, VelocityControlType velocityControlType, ControlSlot controlSlot, ArbitraryFeedforward arbitraryFeedforward) {
        setSetpoint(
            velocity.in(RPM), 
            ((SparkVelocityControlType) velocityControlType).get(), 
            (SparkControlSlot) controlSlot, 
            arbitraryFeedforward.getVolts());
    }

    @Override
    protected void setPosition(Angle position, PositionControlType positionControlType, ControlSlot controlSlot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward) {
        SparkControlSlot sparkControlSlot = (SparkControlSlot) controlSlot;
        setSetpoint(
            position.in(Rotations), 
            ((SparkPositionControlType) positionControlType).get(), 
            sparkControlSlot, 
            velocity.in(RPM) * slotControlConfigs[sparkControlSlot.getIndex()].ffConfig.kV + arbitraryFeedforward.getVolts());
    }

    @Override
    protected Angle getPosition() {
        return Rotations.of(encoder.getPosition());
    }

    @Override
    protected AngularVelocity getVelocity() {
        return RPM.of(encoder.getVelocity());
    }

    @Override
    protected Voltage getVoltage() {
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    protected Current getStatorCurrent() {
        return Amps.of(motor.getOutputCurrent());
    }

    @Override
    protected Temperature getTemperature() {
        return Celsius.of(motor.getMotorTemperature());
    }

    @Override
    public void applyConfigRequests(ConfigRequests configRequests) {
        if (!configRequests.configChanged()) {
            return;
        }

        SparkConfig config = (SparkConfig) configRequests.getConfig();

        slotControlConfigs[0] = config.closedLoopConfig.slot0Config.controlConfig;
        slotControlConfigs[1] = config.closedLoopConfig.slot1Config.controlConfig;
        slotControlConfigs[2] = config.closedLoopConfig.slot2Config.controlConfig;
        slotControlConfigs[3] = config.closedLoopConfig.slot3Config.controlConfig;

        vendorConfig.inverted(config.inverted);
        vendorConfig.smartCurrentLimit((int) config.smartCurrentStallLimit.in(Amps));
        vendorConfig.idleMode(config.neutralMode == NeutralMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
        vendorConfig.voltageCompensation(config.voltageCompensation.in(Volts));

        configureFeedbackSensor(config);

        configureSignals(config.signalsConfig);

        /* Soft Limit */
        vendorConfig.softLimit.forwardSoftLimitEnabled(config.softLimitConfig.forwardSoftLimitEnabled);
        vendorConfig.softLimit.forwardSoftLimit(config.softLimitConfig.forwardSoftLimit.in(Rotations));
        vendorConfig.softLimit.reverseSoftLimitEnabled(config.softLimitConfig.reverseSoftLimitEnabled);
        vendorConfig.softLimit.reverseSoftLimit(config.softLimitConfig.reverseSoftLimit.in(Rotations));

        /* Closed Loop */
        vendorConfig.closedLoop.feedbackSensor(
            switch (config.closedLoopConfig.feedbackSensor) {
                case PRIMARY_ENCODER -> FeedbackSensor.kPrimaryEncoder;
                case ABSOLUTE_ENCODER -> FeedbackSensor.kAbsoluteEncoder;
            }
        );
        vendorConfig.closedLoop.positionWrappingEnabled(config.closedLoopConfig.positionContinuousWrap);
        vendorConfig.closedLoop.positionWrappingMinInput(config.closedLoopConfig.positionWrappingMinInput.in(Rotations));
        vendorConfig.closedLoop.positionWrappingMaxInput(config.closedLoopConfig.positionWrappingMaxInput.in(Rotations));
        configureClosedLoopSlot(config.closedLoopConfig.slot0Config);
        configureClosedLoopSlot(config.closedLoopConfig.slot1Config);
        configureClosedLoopSlot(config.closedLoopConfig.slot2Config);
        configureClosedLoopSlot(config.closedLoopConfig.slot3Config);

        /* Follower */
        if (config.followerConfig.isFollower) {
            vendorConfig.follow(config.followerConfig.leaderCanId, config.followerConfig.invertFollowerOutput);
        } else {
            vendorConfig.disableFollowerMode();
        }

        /* Apply Config */
        if (!initialConfigApplied) {
            motor.configure(vendorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            initialConfigApplied = true;
        } else {
            motor.configureAsync(vendorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    private void configureClosedLoopSlot(SlotConfig slotConfig) {
        ClosedLoopSlot slot = slotConfig.controlSlot.get();

        vendorConfig.closedLoop.p(slotConfig.controlConfig.pidConfig.kP, slot);
        vendorConfig.closedLoop.i(slotConfig.controlConfig.pidConfig.kP, slot);
        vendorConfig.closedLoop.d(slotConfig.controlConfig.pidConfig.kP, slot);

        vendorConfig.closedLoop.feedForward.kS(slotConfig.controlConfig.ffConfig.kS, slot);
        vendorConfig.closedLoop.feedForward.kV(slotConfig.controlConfig.ffConfig.kV, slot);
        vendorConfig.closedLoop.feedForward.kA(slotConfig.controlConfig.ffConfig.kA, slot);
        vendorConfig.closedLoop.feedForward.kG(slotConfig.controlConfig.ffConfig.kG, slot);
        vendorConfig.closedLoop.feedForward.kCos(slotConfig.controlConfig.ffConfig.kCos, slot);

        vendorConfig.closedLoop.allowedClosedLoopError(slotConfig.allowedMechanismClosedLoopError.in(Rotations), slot);

        vendorConfig.closedLoop.maxMotion.cruiseVelocity(slotConfig.maxMotionMechanismCruiseVelocity.in(RPM), slot);
        vendorConfig.closedLoop.maxMotion.maxAcceleration(slotConfig.maxMotionMechanismMaxAcceleration.in(RPM.per(Second)), slot);
    }

    private void configureFeedbackSensor(SparkConfig config) {
        double conversionFactor = 1.0 / config.sensorToMechanismGearbox.getReduction();

        vendorConfig.encoder.positionConversionFactor(conversionFactor);
        vendorConfig.encoder.velocityConversionFactor(conversionFactor);
        vendorConfig.encoder.uvwMeasurementPeriod(config.encoderConfig.uvwMeasurementPeriodMs);
        vendorConfig.encoder.uvwAverageDepth(config.encoderConfig.uvwMeasurementPeriodMs);

        vendorConfig.absoluteEncoder.inverted(config.inverted);
        vendorConfig.absoluteEncoder.positionConversionFactor(conversionFactor);
        vendorConfig.absoluteEncoder.velocityConversionFactor(conversionFactor);
        vendorConfig.absoluteEncoder.averageDepth(config.absoluteEncoderConfig.averageDepth);
    }

    private void configureSignals(SignalsConfig signalsConfig) {
        vendorConfig.signals.primaryEncoderPositionAlwaysOn(signalsConfig.primaryEncoderPositionAlwaysOn);
        vendorConfig.signals.primaryEncoderPositionPeriodMs(signalsConfig.primaryEncoderPositionPeriodMs);
        vendorConfig.signals.primaryEncoderVelocityAlwaysOn(signalsConfig.primaryEncoderVelocityAlwaysOn);
        vendorConfig.signals.primaryEncoderVelocityPeriodMs(signalsConfig.primaryEncoderVelocityPeriodMs);

        vendorConfig.signals.absoluteEncoderPositionAlwaysOn(signalsConfig.absoluteEncoderPositionAlwaysOn);
        vendorConfig.signals.absoluteEncoderPositionPeriodMs(signalsConfig.absoluteEncoderPositionPeriodMs);
        vendorConfig.signals.absoluteEncoderVelocityAlwaysOn(signalsConfig.absoluteEncoderVelocityAlwaysOn);
        vendorConfig.signals.absoluteEncoderVelocityPeriodMs(signalsConfig.absoluteEncoderVelocityPeriodMs);

        vendorConfig.signals.appliedOutputPeriodMs(signalsConfig.appliedOutputPeriodMs);
        vendorConfig.signals.busVoltagePeriodMs(signalsConfig.busVoltagePeriodMs);
        vendorConfig.signals.outputCurrentPeriodMs(signalsConfig.outputCurrentPeriodMs);
    }

    @Override
    public void setEncoderPosition(Angle position) {
        encoder.setPosition(position.in(Rotations));
    }

    @Override
    public void clearStickyFaults() {
        motor.clearFaults();
    }
    
    /** 
     * Feedforward unit can be voltage or percent out, but currently, this method only works for voltage.
     * If you need percent out for whatever reason, you can modify the parameter(s) or create another method.
     */
    private void setSetpoint(double setpoint, ControlType controlType, SparkControlSlot controlSlot, double feedforwardVolts) {
        controller.setSetpoint(setpoint, controlType, controlSlot.get(), feedforwardVolts, ArbFFUnits.kVoltage);
    }
}