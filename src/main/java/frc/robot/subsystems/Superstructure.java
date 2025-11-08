package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ArmPivotConstants;
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

  @AutoLogOutput(key = "RobotStates/Extention State")
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

  @AutoLogOutput(key = "RobotStates/Arm Level")
  private int pivotLevel = 2; // Selected Reef Level

  private int returnPivotLevel() {
    return pivotLevel;
  }

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

  /** Extends the claw */
  public Command extendClaw() {
    return Commands.sequence(
        armext.setSpeed(0.23),
        Commands.waitSeconds(1.1),
        armext.setSpeed(0),
        Commands.runOnce(
            () -> {
              extentionState = "extended";
            }));
  }

  /** Retracts the claw */
  public Command retractClaw() {
    return Commands.sequence(
        armext.setSpeed(-0.2),
        Commands.waitSeconds(1.1),
        armext.setSpeed(0),
        Commands.runOnce(
            () -> {
              extentionState = "retracted";
            }));
  }

  /** Grabs the coral from the tray */
  public Command ready() {
    return Commands.sequence(
        claw.changeSetpoint(-1.0),
        extendClaw(),
        Commands.waitSeconds(0.25),
        claw.changeSetpoint(0),
        retractClaw());
  }

  /**
   * Selects the height to raise the arm to when commanded.
   *
   * @param height Level to score on [1,2,3]
   */
  public Command selectPivotHeight(int height) {
    return Commands.runOnce(() -> pivotLevel = height)
        .andThen(logMessage("Selected Pivot Height: " + height));
  }

  /** Raise the arm to the height selected by selectPivotHeight */
  public Command raisePivot() {
    return Commands.sequence(
            retractClaw().onlyIf(() -> (extentionState == "extended")),
            pivot.changeSetpoint(() -> pivotSetpoints[pivotLevel]))
        .andThen(
            logMessage(
                "Pivot Setpoint Changed To: "
                    + pivotSetpoints[pivotLevel]
                    + " Reef Level: "
                    + pivotLevel));
  }

  /** Homes the arm and stops all motors */
  public Command homeRobot() {
    return Commands.sequence(
        hopper.changeSetpoint(0),
        claw.changeSetpoint(0),
        pivot.changeSetpoint(Constants.ArmPivotConstants.initialSetpoint));
  }

  /** Runs the hopper and moves the arm out of the way */
  public Command runIntake() {
    return Commands.sequence(pivot.changeSetpoint(-75), hopper.changeSetpoint(-1.0));
  }

  /** Stops the intake and puts the arm back */
  public Command stopIntake() {
    return Commands.sequence(
        hopper.changeSetpoint(0),
        pivot.changeSetpoint(ArmPivotConstants.initialSetpoint),
        Commands.waitUntil(pivot::atSetpoint)
        // ready() //Might be good if the hopper cant jam
        );
  }

  /** Score the coral on the reef pole, or drop onto l1 */
  public Command Score() {
    return Commands.sequence(
        logMessage("Score"),
        Commands.either(
            Commands.sequence(
                claw.changeSetpoint(1.0), Commands.waitSeconds(0.5), claw.changeSetpoint(0)),
            pivot.changeSetpoint(() -> pivotSetpoints[pivotLevel + 3]),
            () -> pivotLevel == 1));
  }

  public Command Taxi() {
    return Commands.sequence(
        drivebase.runVelocityFieldRelative(() -> new ChassisSpeeds(0.5, 0, 0)).withTimeout(3.5),
        drivebase.runVelocityFieldRelative(() -> new ChassisSpeeds(0, 0, 0)));
  }
}
