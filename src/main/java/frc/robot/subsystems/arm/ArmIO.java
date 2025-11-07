package frc.robot.subsystems.arm;

import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        private boolean connected = false;
        private boolean encoderConnected = false;
        private double velocity = 0;
        private double position = 0;
        private double currentAmps = 0;
        private double currentVoltage = 0;
        private double temperature = 0;
    }
    /* Update the loggable inputs*/
    default void updateInputs(ModuleIOInputs inputs) {}
}
