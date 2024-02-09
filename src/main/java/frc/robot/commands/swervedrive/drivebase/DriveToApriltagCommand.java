package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.Utility;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;

/*
public class DriveToApriltagCommand extends Command {
  private double m_tyOffset;
  private double m_rotationGoal;
  private int m_pipelineIndex;
  private Debouncer m_canSeePieceDebouncer;
  private final SwerveSubsystem  swerve;
  private final SwerveController controller;
  private TeleopDrive drive;

  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;

  public DriveToApriltagCommand(SwerveSubsystem swerve, double tyOffset, double rotationGoal, int pipelineIndex) {
    m_tyOffset = tyOffset;
    m_rotationGoal = rotationGoal;
    m_pipelineIndex = pipelineIndex;
    this.swerve= swerve;
    this.controller = swerve.getSwerveController();

    m_xController = DriveConstants.APRILTAG_X_TRANSLATION_PID_CONTROLLER;
    m_yController = DriveConstants.APRILTAG_Y_TRANSLATION_PID_CONTROLLER;
    m_rotationController = DriveConstants.APRILTAG_ROTATION_PID_CONTROLLER;

    addRequirements(swerve);

    //addRequirements(Robot.swerve);
  }

  private void drive (DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega){
    this.drive = new TeleopDrive(this.swerve, vX, vY, omega, ()->false);
  }

  @Override
  public void initialize() {
    Robot.shooterLimelight.setPipeline(m_pipelineIndex);

    // reset pids
    m_yController.setSetpoint(m_tyOffset + DriveConstants.APRILTAG_TY_MAGIC_OFFSET);
    m_yController.reset();

    m_xController.setSetpoint(DriveConstants.AMP_TX_SETPOINT);
    m_xController.reset();

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.setSetpoint(m_rotationGoal);
    m_rotationController.reset();

    m_canSeePieceDebouncer =
        new Debouncer(DriveConstants.AUTO_TRANSLATE_DEBOUNCE_SECONDS, DebounceType.kFalling);
  }

  @Override
  public void execute() {
    if (!m_canSeePieceDebouncer.calculate(Robot.shooterLimelight.validTargetExists())) {
      System.out.println("No Target Detected");
      swerve.drive(new Translation2d(0,0),0, false);
      return;
    }

    double tx = Robot.shooterLimelight.getTX();
    double ty = Robot.shooterLimelight.getTY();
    double rotationError = swerve.getPose().getRotation().getRadians();

    // Increase tx tolerance when close to target since tx is more sensitive at
    // shorter distances
    double txTolerance = DriveConstants.APRILTAG_X_TOLERANCE;
    if (Utility.isWithinTolerance(ty, m_tyOffset, 4)) {
      txTolerance *= 2;
    }

    if (Utility.isWithinTolerance(rotationError, 0, DriveConstants.APRILTAG_ROTATION_TOLERANCE)) {
      rotationError = 0;
    }

    // Reduce tx based on how far off our rotation is so the x controller doesn't
    // over compensate
    tx -= Rotation2d.fromRadians(rotationError).getDegrees();
    if (Utility.isWithinTolerance(tx, 0, txTolerance)) {
      tx = 0;
    }
    if (Utility.isWithinTolerance(ty, m_tyOffset, DriveConstants.APRILTAG_Y_TOLERANCE)) {
      ty = m_tyOffset;
    }
    //Jacob added this 2/7/24
    final double tx_final = tx;
    final double ty_final = ty;
    final double rotationError_final = rotationError;

    drive(
        ()->-m_yController.calculate(ty_final + DriveConstants.APRILTAG_TY_MAGIC_OFFSET),
        ()->m_xController.calculate(tx_final),
        ()->m_rotationController.calculate(rotationError_final));
  }

  @Override
  public void end(boolean interrupted) {
    drive(()->0, ()->0, ()->0);
  }
}*/