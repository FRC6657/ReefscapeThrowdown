package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO io) {
    this.io = io;
  }

  /**
   * Sets the speed of the hopper.
   *
   * @param setpoint The speed setpoint for the hopper. (-1,1)
   * @return A command that sets the speed of the hopper.
   */
  public Command changeSetpoint(double setpoint) {
    return this.runOnce(
        () -> {
          io.setSpeed(setpoint);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }
}
