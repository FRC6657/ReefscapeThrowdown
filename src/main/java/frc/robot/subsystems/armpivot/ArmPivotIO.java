package frc.robot.subsystems.armpivot;

import org.littletonrobotics.junction.AutoLog;

public interface ArmPivotIO {

    @AutoLog 
    public static class ArmPivotIOInputs {
        public double kSetpoint = 0.0;
        public double kVelocity = 0.0;
        public double kTemp = 0.0;
        public double kVoltage = 0.0;
        public double kCurrent = 0.0;
    }

    public default void updateInputs(ArmPivotIOInputs inputs) {}

    public default boolean getBeamBroken(){
        return true;
    }

    public default void changeSetpoint(double setpoint) {}
} 
