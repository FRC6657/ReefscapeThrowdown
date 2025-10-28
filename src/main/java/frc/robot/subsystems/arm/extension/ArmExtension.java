// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.extension;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtension extends SubsystemBase {

  public final ArmExtensionIO io;
  public final ArmExtensionIOInputsAutoLogged inputs = new ArmExtensionIOInputsAutoLogged();

  /** Creates a new armExtension. */
  public ArmExtension(ArmExtensionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Arm Extension", inputs);
  }

  public Command setSpeed(double dutycyc) {
    return this.runOnce(() -> io.setSpeed(dutycyc)); // speed from -1 to 1
  }
}
