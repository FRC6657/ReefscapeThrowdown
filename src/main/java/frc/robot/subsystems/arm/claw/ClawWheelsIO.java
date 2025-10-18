package frc.robot.subsystems.arm.claw;


import org.littletonrobotics.junction.AutoLog;

public interface ClawWheelsIO{
    
    @AutoLog
    public static class ClawWheelsIOInputs{
        //TODO: k is usually a prefix for constants. Change name.
        public double kSetpoint = 0.0;
        public double kTemp = 0.0; //TODO: Probably dont need to log this
        public double kVoltage = 0.0;
        public double kCurrent = 0.0; //TODO: Probably dont need to log this
    }

    public default void updateInputs(ClawWheelsIOInputs inputs){}

    /**
     * Sets the speed of the claw wheels.
     * @param speed The speed setpoint for the claw wheels. (-1,1)
     */
    public default void setSpeed(double speed){}

}

