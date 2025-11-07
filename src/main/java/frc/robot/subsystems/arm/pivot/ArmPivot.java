package frc.robot.subsystems.arm.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ArmPivot extends SubsystemBase {

  private final ArmPivotIO io;
  private final ArmPivotIOInputsAutoLogged inputs = new ArmPivotIOInputsAutoLogged();

  public ArmPivot(ArmPivotIO io) {
    this.io = io;
  }

  public Command changeSetpoint(double setpoint) {
    return this.runOnce(
        () -> {
          io.changeSetpoint(setpoint);
        });
  }

  public Command changeSetpoint(DoubleSupplier setpoint) {
    return changeSetpoint(setpoint.getAsDouble());
  }

  public void setpoint(double setpoint) {
    io.changeSetpoint(setpoint);
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(inputs.kSetpoint, inputs.kPosition, 2);
  }

  public double getPosition() {
    return inputs.kPosition;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ArmPivot", inputs);
  }
}
