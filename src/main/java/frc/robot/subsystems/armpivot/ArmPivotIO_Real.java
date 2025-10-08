package frc.robot.subsystems.armpivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmPivotIO_Real implements ArmPivotIO {

    // Pivot Motor Controller
    TalonFX pivotMotor = new TalonFX(Contants.CAN.kPivot.id);

    // Remember unit circle!!
    private double kSetpoint = -90; 

   public ArmPivotIO_Real() {

    var motorConfigurator = pivotMotor.getConfigurator();
    var motorConfigs = new TalonFXConfiguration();
    motorConfigs.Feedback.SensorToMechanismRatio = 
       1.0 / Constants.ArmPivot.gearRatio;
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
    motorConfigurator.apply(motorConfigs);


   }
    
}
