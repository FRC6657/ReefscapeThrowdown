package frc.robot.subsystems.clawwheels;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class ClawWheels extends SubsystemBase{

    private final ClawWheelsIO io;
    private final ClawWheelsIOInputsAutoLogged inputs = new ClawWheelsIOInputsAutoLogged();

    public ClawWheels(ClawWheelsIO io){
        this.io = io;
    }

    public Command changeSetpoint(double setpoint){
        return this.runOnce( () -> {
            io.setSpeed(setpoint);
        });
    }

    public void setspeed(double setpoint) {
        io.setSpeed(setpoint); // percentage
      }

     @Override
        public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ClawWheels", inputs);
      }



    

    
    
    

    
}
