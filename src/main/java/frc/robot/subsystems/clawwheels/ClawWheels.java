package frc.robot.subsystems.clawwheels;

import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class ClawWheels extends SubsystemBase{

    private final ClawWheelsIO io;
    private final ClawWheelsIOInputsAutoLogged inputs = new ClawWheelsIOInputsAutoLogged();

    public ClawWheels(ClawWheelsIO io){
        this.io = io;
    }



    

    
    
    

    
}
