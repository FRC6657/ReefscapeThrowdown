package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CodeConstants;
import frc.robot.Constants.MAXSwerveConstants;
import edu.wpi.first.math.system.plant.LinearSystemId;


public class MAXSwerveIO_Sim implements MAXSwerveIO {

  private DCMotorSim driveSim = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getNEO(1), 
      0.005, 
      MAXSwerveConstants.kDriveMotorReduction
    ),
    DCMotor.getNEO(1), 
    0,
    0
  );

  private DCMotorSim turnSim = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getNeo550(1), 
      0.0004, 
      MAXSwerveConstants.kTurnMotorReduction
    ),
    DCMotor.getNeo550(1), 
    0,
    0
  );

  private final Rotation2d turnAbsoluteInitialPosition =
    new Rotation2d(Math.random() * 2 * Math.PI);

  private double driveVolts = 0.0;
  private double turnVolts = 0.0;

  private double mpsSetpoint = 0.0;
  private Rotation2d turnAngleSetpoint = new Rotation2d();

  private PIDController turnController = new PIDController(15, 0.0, 0.0);
  private PIDController driveController = new PIDController(5, 0.0, 0.0);
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, MAXSwerveConstants.kDriveFF * 12);

  public MAXSwerveIO_Sim() {
    // Set the range of the turn motor
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    // Set the initial position of the turn motor
    turnSim.setState(turnAbsoluteInitialPosition.getRadians(), 0);
  }

  @Override
  public void updateInputs(MAXSwerveIOInputs inputs) {

    setDriveVoltage(
        driveFeedforward.calculate(mpsSetpoint)
            + driveController.calculate(
                (driveSim.getAngularVelocityRPM() * MAXSwerveConstants.kWheelCircumferenceMeters)
                    / 60,
                mpsSetpoint));
    setTurnVoltage(
        turnController.calculate(getTurnAngle().getRadians(), turnAngleSetpoint.getRadians()));

    // Step the simulation forward
    driveSim.update(1 / CodeConstants.kMainLoopFrequency);
    turnSim.update(1 / CodeConstants.kMainLoopFrequency);

    // Update the inputs
    inputs.drivePositionMeters =
        driveSim.getAngularPositionRotations() * MAXSwerveConstants.kWheelCircumferenceMeters;
    inputs.driveVelocityMPS =
        (driveSim.getAngularVelocityRPM() * MAXSwerveConstants.kWheelCircumferenceMeters) / 60;
    inputs.driveAppliedVolts = driveVolts;
    inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();

    inputs.turnPositionRad = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnVolts;
    inputs.turnCurrentAmps = turnSim.getCurrentDrawAmps();
  }

  /** Sets the drive motor voltage */
  public void setDriveVoltage(double volts) {
    driveVolts = MathUtil.clamp(volts, -12, 12);
    driveSim.setInputVoltage(volts);
  }

  /** Sets the turn motor voltage */
  public void setTurnVoltage(double volts) {
    turnVolts = MathUtil.clamp(volts, -12, 12);
    turnSim.setInputVoltage(turnVolts);
  }

  /** Sets the drive MPS setpoint */
  @Override
  public void setDriveMPS(double mps) {
    mpsSetpoint = mps;
  }

  /** Sets the turn angle setpoint */
  @Override
  public void setTurnAngle(Rotation2d angle) {

    turnAngleSetpoint = angle;
  }

  /** Gets the turn angle */
  @Override
  public Rotation2d getTurnAngle() {
    return Rotation2d.fromRadians(turnSim.getAngularPositionRad());
  }
}
