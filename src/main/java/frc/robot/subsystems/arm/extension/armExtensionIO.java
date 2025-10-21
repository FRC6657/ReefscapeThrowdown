package frc.robot.subsystems.arm.extension;

import org.littletonrobotics.junction.AutoLog;

public interface armExtensionIO {
    @AutoLog
    public static class armExtensionIOInputs {
        public double volts = 0;
    }
    
    public default void updateInputs(armExtensionIOInputs inputs) {}
    public default void setSpeed(double dutycyc) {}
}
