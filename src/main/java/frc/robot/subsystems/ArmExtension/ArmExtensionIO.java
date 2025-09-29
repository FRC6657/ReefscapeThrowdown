package frc.robot.subsystems.armextension;

import org.littletonrobotics.junction.AutoLog;

public interface ArmExtensionIO {
    @AutoLog
    public static class ArmExtensionIOInputs {
        public double volts = 0;
        public double amps = 0;
    }
    
    public default void updateInputs(ArmExtensionIOInputs inputs) {}
    public default void setVoltage(double volts) {}
}
