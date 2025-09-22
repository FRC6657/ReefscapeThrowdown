package frc.robot.subsystems.armExtension;

import org.littletonrobotics.junction.AutoLog;

public interface armExtensionIO {
    @AutoLog
    public static class armExtensionIOInputs {
        public double temp = 0; // in Celcius
        public double volts = 0;
        public double amps = 0;
    }
    
    public default void updateInputs(armExtensionIOInputs inputs) {}
    public default void setVoltage(double volts) {}
}
