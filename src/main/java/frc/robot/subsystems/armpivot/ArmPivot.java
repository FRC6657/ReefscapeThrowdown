package frc.robot.subsystems.armpivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmPivot extends SubsystemBase {
    
    private final ArmPivot io;
    private final ArmPivotIOInputsAutoLogged inputs = new ArmPivotIOInputsAutoLogged();

    public ArmPivot(ArmPivotIO io) {
        this.io = io;
    }

    public Command changeSetpoint(double setpoint) {
        return this.runOnce(
            () -> {
                io.changeSetpoint(setpoint);
            }
        );
    }

    public void setpoint(double setpoint) {
        io.changeSetpoint(setpoint);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ArmPivot", inputs);
    }
}
