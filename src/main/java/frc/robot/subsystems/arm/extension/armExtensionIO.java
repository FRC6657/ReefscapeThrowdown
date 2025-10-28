package frc.robot.subsystems.arm.extension;

import org.littletonrobotics.junction.AutoLog;

public interface ArmExtensionIO {
    @AutoLog
    public static class ArmExtensionIOInputs {
        public double volts = 0;
    }
    
    public default void updateInputs(ArmExtensionIOInputs inputs) {}
    public default void setSpeed(double dutycyc) {}
}
