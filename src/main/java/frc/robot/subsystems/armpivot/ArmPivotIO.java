package frc.robot.subsystems.armpivot;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants;

public interface ArmPivotIO {

    @AutoLog 
    public static class ArmPivotIOInputs {
        public double kSetpoint = 0.0;
        public double kVelocity = 0.0;
        public double kTemp = 0.0;
        public double kVoltage = 0.0;
        public double kCurrent = 0.0;
        public double kPosition = Constants.ArmPivotConstants.kInitialSetpoint;
    }

    public default void updateInputs(ArmPivotIOInputs inputs) {}

    public default void changeSetpoint(double setpoint) {}
} 
