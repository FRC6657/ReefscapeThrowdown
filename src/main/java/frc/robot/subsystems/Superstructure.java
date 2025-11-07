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
import frc.robot.subsystems.hopper.Hopper;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  private MAXSwerve drivebase;
  private ArmExtension armext;
  private ClawWheels claw;
  private ArmPivot pivot;
  private Hopper hopper;

  private String extentionState = "retracted";

  // Array for easily grabbing setpoint angles.
  private double[] pivotSetpoints = {
    Constants.ArmPivotConstants.initialSetpoint,
    -17, // L1 Ready
    12, // L2 Ready
    55, // L3 Ready
    -17, // L1 Score
    -25, // L2 Score
    17, // L3 Score
  };

  @AutoLogOutput(key = "RobotStates/Elevator Level")
  private int pivotLevel = 2; // Selected Reef Level

  public Superstructure(
      MAXSwerve drivebase, ArmExtension armext, ClawWheels claw, ArmPivot pivot, Hopper hopper) {
    this.drivebase = drivebase;
    this.armext = armext;
    this.claw = claw;
    this.pivot = pivot;
    this.hopper = hopper;
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

  public Command extendClaw() {
    return Commands.sequence(
        armext.setSpeed(0.2),
        Commands.waitSeconds(0.5),
        armext.setSpeed(0),
        Commands.runOnce(
            () -> {
              extentionState = "extended";
            }));
  }

  public Command retractClaw() {
    return Commands.sequence(
        armext.setSpeed(-0.2),
        Commands.waitSeconds(0.5),
        armext.setSpeed(0),
        Commands.runOnce(
            () -> {
              extentionState = "retracted";
            }));
  }

  public Command ready() {
    return Commands.sequence(
        claw.changeSetpoint(-0.3), extendClaw(), claw.changeSetpoint(0), retractClaw());
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
                "Pivot Setpoint Changed To: "
                    + pivotSetpoints[pivotLevel]
                    + " Reef Level: "
                    + pivotLevel));
  }

  public Command HomeRobot() {
    return Commands.sequence(
        armext.setSpeed(0),
        claw.changeSetpoint(0),
        pivot.changeSetpoint(Constants.ArmPivotConstants.initialSetpoint));
  }

  public Command Intake() {
    return Commands.sequence(
        hopper.changeSetpoint(0.5), Commands.waitSeconds(1), hopper.changeSetpoint(0));
  }

  public Command Score() {
    return Commands.sequence(
        logMessage("Score"),
        Commands.either(
        Commands.sequence(
          claw.changeSetpoint(0.3),
          Commands.waitSeconds(0.5),
          claw.changeSetpoint(0)
        ),
        pivot.changeSetpoint(() -> pivotSetpoints[pivotLevel+3]),
        () -> pivotLevel == 1)
        );
  }
}
