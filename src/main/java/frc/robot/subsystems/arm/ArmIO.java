package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public double velocity = 0;
        public double position = 0;
        public double currentAmps = 0;
        public double currentVoltage = 0;
        public double temperature = 0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
}
