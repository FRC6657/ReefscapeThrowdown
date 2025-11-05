package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.arm.claw.ClawWheels;
import frc.robot.subsystems.arm.extension.ArmExtension;
import frc.robot.subsystems.arm.pivot.ArmPivot;
import frc.robot.subsystems.drive.MAXSwerve;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  private MAXSwerve drivebase;
  private ArmExtension armext;
  private ClawWheels claw;
  private ArmPivot pivot;

  private String extentionState = "retracted";

  // Array for easily grabbing setpoint angles.
  private double[] pivotSetpoints = {
    Constants.ArmPivotConstants.initialSetpoint,
    Units.inchesToMeters(0), // L1
    Units.inchesToMeters(16.5), // L2
    Units.inchesToMeters(31.5), // L3
  };

  @AutoLogOutput(key = "RobotStates/Elevator Level")
  private int pivotLevel = 2; // Selected Reef Level

  public Superstructure(MAXSwerve drivebase, ArmExtension armext, ClawWheels claw, ArmPivot pivot) {
    this.drivebase = drivebase;
    this.armext = armext;
    this.claw = claw;
    this.pivot = pivot;
  }

  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
  }

  @AutoLogOutput(key = "Superstructure/RobotPoses")
  public Pose3d[] getRobotPoses() {
    double fakeExtentionDistance = extentionState == "extended" ? 3.25 : 0;
    return new Pose3d[] {
      new Pose3d(
          0.184150,
          0,
          0.822649,
          new Rotation3d(0, -Units.degreesToRadians(pivot.getPosition()) - Math.PI / 2, 0.0)),
      new Pose3d(
          0.184150
              + Math.cos(Units.degreesToRadians(pivot.getPosition()))
                  * Units.inchesToMeters(fakeExtentionDistance),
          0,
          0.822649
              + Math.sin(Units.degreesToRadians(pivot.getPosition()))
                  * Units.inchesToMeters(fakeExtentionDistance),
          new Rotation3d(0, -Units.degreesToRadians(pivot.getPosition()) - Math.PI / 2, 0))
    };
  }

  public Command selectPivotHeight(int height) {
    return Commands.runOnce(() -> pivotLevel = height)
        .andThen(logMessage("Selected Pivot Height: " + height));
  }

  // Change Elevator Setpoint to the selected reef level.
  public Command raisePivot() {
    return pivot
        .changeSetpoint(() -> pivotSetpoints[pivotLevel])
        .andThen(
            logMessage(
                "Elevator Setpoint Changed To: "
                    + pivotSetpoints[pivotLevel]
                    + " Reef Level: "
                    + pivotLevel));
  }

  public Command HomeRobot() {
    return Commands.sequence(Commands.none());
  }
}
