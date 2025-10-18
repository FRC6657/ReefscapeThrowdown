package frc.robot.subsystems.arm.claw;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawWheels extends SubsystemBase {

    private final ClawWheelsIO io;
    private final ClawWheelsIOInputsAutoLogged inputs = new ClawWheelsIOInputsAutoLogged();

    public ClawWheels(ClawWheelsIO io) {
        this.io = io;
    }

    /**
     * Sets the speed of the claw wheels.
     * @param setpoint The speed setpoint for the claw wheels. (-1,1)
     * @return A command that sets the speed of the claw wheels.
     */
    public Command changeSetpoint(double setpoint) {
        return this.runOnce(() -> {
            io.setSpeed(setpoint);
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ClawWheels", inputs);
    }

}
