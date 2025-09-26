package frc.robot.subsystems.clawwheels;


import org.littletonrobotics.junction.AutoLog;

public interface ClawWheelsIO{
    
    @AutoLog
    public static class ClawWheelsIOInputs{
        public double kSetpoint = 0.0;
        public double kTemp = 0.0;
        public double kVoltage = 0.0;
        public double kCurrent = 0.0;
    }

    public default void updateInputs(ClawWheelsIOInputs inputs){}

    public default void setVoltage(double voltage){}

}

