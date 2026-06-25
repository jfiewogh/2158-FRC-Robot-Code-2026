package org.austincans.lib.io.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.austincans.lib.io.motors.MotorIO.PositionControlType;
import org.austincans.lib.io.motors.MotorIO.VelocityControlType;
import org.austincans.lib.io.motors.SparkConfig.SignalsConfig;
import org.austincans.lib.io.motors.SparkConfig.SlotConfig;
import org.austincans.lib.io.motors.SparkIO.SparkControlSlot;
import org.austincans.lib.io.motors.SparkIO.SparkPositionControlType;
import org.austincans.lib.io.motors.SparkIO.SparkVelocityControlType;
import org.austincans.lib.io.motors.configrequests.SparkIOConfigRequests;
import org.austincans.lib.io.motors.configs.ControlConfig;
import org.austincans.lib.io.motors.configs.NeutralMode;
import org.austincans.lib.io.motors.inputs.SparkIOInputs;

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

/*
 * To configure while deployed, use kNoResetSafeParameters and kNoPersistParameters 
 * so that you can configure only specific values while keeping existing ones.
 */
public class SparkIO<
    MotorType extends SparkBase, 
    VendorConfigType extends SparkBaseConfig
> extends AbstractMotorIO<
    SparkIOInputs, 
    SparkControlSlot, 
    SparkVelocityControlType, 
    SparkPositionControlType, 
    SparkIOConfigRequests,
    SparkConfig
> {
    protected final MotorType motor;
    protected final RelativeEncoder encoder;
    protected final SparkClosedLoopController controller;

    private final VendorConfigType vendorConfig;
    private boolean initialConfigApplied = false;

    private final ControlConfig[] slotControlConfigs = new ControlConfig[4];

    protected SparkIO(MotorType motor, VendorConfigType vendorConfig) {
        this.vendorConfig = vendorConfig;

        this.motor = motor;
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(SparkIOInputs inputs) {
        super.updateInputs(inputs);

        Faults activeFaults = motor.getFaults();
        inputs.activeFaultMotorType = activeFaults.motorType;
        inputs.activeFaultCan = activeFaults.can;
        inputs.activeFaultTemperature = activeFaults.temperature;
        inputs.activeFaultGateDriver = activeFaults.gateDriver;
        inputs.activeFaultEscEeprom = activeFaults.escEeprom;
        inputs.activeFaultFirmware = activeFaults.firmware;

        Faults stickyFaults = motor.getStickyFaults();
        inputs.stickyFaultMotorType = stickyFaults.motorType;
        inputs.stickyFaultCan = stickyFaults.can;
        inputs.stickyFaultTemperature = stickyFaults.temperature;
        inputs.stickyFaultGateDriver = stickyFaults.gateDriver;
        inputs.stickyFaultEscEeprom = stickyFaults.escEeprom;
        inputs.stickyFaultFirmware = stickyFaults.firmware;
        inputs.stickyFaultOther = stickyFaults.other;
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
    protected void setVelocity(AngularVelocity velocity, SparkVelocityControlType velocityControlType, SparkControlSlot controlSlot, ArbitraryFeedforward arbitraryFeedforward) {
        setSetpoint(velocity.in(RPM), velocityControlType.get(), controlSlot, arbitraryFeedforward.volts);
    }

    @Override
    protected void setPosition(Angle position, SparkPositionControlType positionControlType, SparkControlSlot controlSlot, AngularVelocity velocity, ArbitraryFeedforward arbitraryFeedforward) {
        double kV = velocity.in(RPM) * slotControlConfigs[controlSlot.getIndex()].ffConfig.kV;
        setSetpoint(position.in(Rotations), positionControlType.get(), controlSlot, kV + arbitraryFeedforward.volts);
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
    public void applyConfigRequests(SparkIOConfigRequests configRequests) {
        if (!configRequests.configChanged()) {
            return;
        }
        configRequests.setConfigChanged(false);

        SparkConfig config = configRequests.getConfig();

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
    
    public static enum SparkPositionControlType implements PositionControlType {
        STANDARD(ControlType.kPosition),
        MAXMOTION(ControlType.kMAXMotionPositionControl);

        private final ControlType controlType;

        private SparkPositionControlType(ControlType controlType) {
            this.controlType = controlType;
        }

        private ControlType get() {
            return controlType;
        }
    }

    public static enum SparkVelocityControlType implements VelocityControlType {
        STANDARD(ControlType.kVelocity),
        MAXMOTION(ControlType.kMAXMotionVelocityControl);

        private final ControlType controlType;

        private SparkVelocityControlType(ControlType controlType) {
            this.controlType = controlType;
        }

        private ControlType get() {
            return controlType;
        }
    }

    public static enum SparkControlSlot implements MotorIO.ControlSlot {
        SLOT_0(ClosedLoopSlot.kSlot0), 
        SLOT_1(ClosedLoopSlot.kSlot1), 
        SLOT_2(ClosedLoopSlot.kSlot2), 
        SLOT_3(ClosedLoopSlot.kSlot3);

        private final ClosedLoopSlot slot;

        private SparkControlSlot(ClosedLoopSlot slot) {
            this.slot = slot;
        }

        public ClosedLoopSlot get() {
            return slot;
        }
        public int getIndex() {
            return ordinal();
        }
    }
}
