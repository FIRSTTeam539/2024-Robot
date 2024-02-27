// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.swervedrive.auto.TurnToAprilTagCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.climber.ClimbSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.utils;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.IntakeSubsystem;
import java.io.File;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.auto.TurnToAprilTagCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveSubsystem m_robotDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve"));
  private final ArmSubsystem m_robotArm = new ArmSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final ClimbSubsystem m_robotClimb = new ClimbSubsystem();
  //private final LimelightSubsystem m_robotLimelight = new LimelightSubsystem("limelight");
  // The driver's controller
  CommandXboxController m_driverController0 = new CommandXboxController(OIConstants.kDriverControllerPort0);
  CommandXboxController m_driverController1 = new CommandXboxController(OIConstants.kDriverControllerPort1);
  //SequentialCommandGroup x = new SequentialCommandGroup(new TurnToAprilTagCommand(m_robotDrive, m_robotLimelight), m_robotArm.moveToPositionCommand());
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
     //m_robotDrive.addVisionMeasurement(m_robotLimelight);
    // Configure the button bindings
    configureButtonBindings();
    // Configure default commands
    
    TeleopDrive teleopDrive = new TeleopDrive(m_robotDrive, 
      (()->MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.LEFT_Y_DEADBAND_1)*(OIConstants.kDefaultDriveSpeed+
      OIConstants.kDriveSpeedIncreaseConstant*MathUtil.applyDeadband(m_driverController0.getRightTriggerAxis(), OIConstants.RIGHT_TRIGGER_DEADBAND_1))), 
      (()->MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.LEFT_X_DEADBAND_1)*(OIConstants.kDefaultDriveSpeed+
      OIConstants.kDriveSpeedIncreaseConstant*MathUtil.applyDeadband(m_driverController0.getRightTriggerAxis(), OIConstants.RIGHT_TRIGGER_DEADBAND_1))),  
      (()->MathUtil.applyDeadband(m_driverController0.getRightX(), OIConstants.RIGHT_X_DEADBAND_1)*(OIConstants.kDefaultDriveSpeed+
      OIConstants.kDriveSpeedIncreaseConstant*m_driverController0.getRightTriggerAxis())), 
      ()->true);
    
    /*TeleopDrive simClosedFieldRel = new TeleopDrive(m_robotDrive,
      (()->MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.LEFT_Y_DEADBAND_1)), 
      (()->MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.LEFT_X_DEADBAND_1)), 
      (()->MathUtil.applyDeadband(m_driverController0.getRightX(), OIConstants.LEFT_X_DEADBAND_1)), 
      ()->true);*/
  
    //m_robotDrive.setDefaultCommand(!RobotBase.isSimulation() ? teleopDrive : simClosedFieldRel);
    m_robotDrive.setDefaultCommand(teleopDrive);
    m_robotClimb.setDefaultCommand(Commands.run(()->m_robotClimb.setDualClimb(MathUtil.applyDeadband(m_driverController1.getLeftY(), 0.2)*0.2, MathUtil.applyDeadband(m_driverController1.getRightY(), 0.2)*0.2), m_robotClimb)); // works
    m_robotArm.setDefaultCommand(Commands.run(()->m_robotArm.setArmVelocity(MathUtil.applyDeadband(m_driverController1.getRightTriggerAxis(), 0.1)-MathUtil.applyDeadband(m_driverController1.getLeftTriggerAxis(),0.1)),m_robotArm));

    m_robotIntake.setDefaultCommand(Commands.run(()->m_robotIntake.disable(), m_robotIntake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //when the right stick is pushed down, moves wheels in x formation to stop all movement
    m_driverController0.x().whileTrue(Commands.run(() -> m_robotDrive.lock())); 
    m_driverController0.y().whileTrue(Commands.run(() -> m_robotDrive.zeroGyro()));
    m_driverController0.leftBumper().whileTrue(m_robotArm.moveToPosCommand(0.03));
    m_driverController0.rightBumper().whileTrue(m_robotArm.moveToPosCommand(Math.PI/2));
    
    m_driverController1.leftBumper().whileTrue(Commands.run(()->m_robotClimb.setDualClimb(MathUtil.applyDeadband(m_driverController1.getLeftY(), 0.2), MathUtil.applyDeadband(m_driverController1.getRightY(), 0.2)*0.6), m_robotClimb));
    m_driverController1.rightBumper().whileTrue(m_robotIntake.shootSpeakerCommand());
    m_driverController1.y().whileTrue(m_robotIntake.intakeCommand());
    m_driverController1.x().whileTrue(m_robotIntake.shootAmpCommand());
    //m_driverController1.a().onTrue(m_robotArm.disableArm());
    m_driverController1.povUp().whileTrue(Commands.run(()->m_robotArm.setArmVelocity(MathUtil.applyDeadband(m_driverController1.getRightTriggerAxis(), 0.1)-MathUtil.applyDeadband(m_driverController1.getLeftTriggerAxis(),0.1)*0.25), m_robotArm));
    
    m_driverController1.a().whileTrue(m_robotIntake.justShootCommand(0.5));//MathUtil.applyDeadband(m_driverController1.getLeftTriggerAxis(),0.1)*0.5));
    m_driverController1.b().whileTrue(m_robotIntake.justIntakeCommand(0.5));
    m_driverController1.povDown().whileTrue(m_robotIntake.justIntakeCommand(-0.1));

    
    //m_driverController1.x().whileTrue(m_robotIntake.justIntakeCommand(0));
    //m_driverController1.b().whileTrue(m_robotIntake.justShootCommand(0));

    //m_driverController1.povLeft().whileTrue(m_robotArm.moveToPosCommand(Math.PI/4));

    //m_driverController1.x().whileTrue(m_robotClimb.climbLeftCommand(0.2));
    //m_driverController1.b().whileTrue(m_robotClimb.climbRightCommand(0.2));
    //m_driverController1.y().whileTrue(m_robotClimb.climbRightCommand(-0.2));
    //m_driverController1.leftBumper().whileTrue(m_robotClimb.climbLeftCommand(-0.2));
   
    //m_driverController1.x().whileTrue(m_robotArm.moveArm(0.3));
    //m_driverController1.b().whileTrue(m_robotArm.moveArm(-0.2));
    
    //m_driverController1.leftBumper().whileTrue(Commands.run((()->m_robotArm.moveArmAtSpeed(0.2)), m_robotArm));
    //m_driverController1.rightBumper().whileTrue(Commands.run((()->m_robotArm.moveArmAtSpeed(-0.2)), m_robotArm));
    //m_driverController1.leftTrigger().onTrue(m_robotIntake.intakeCommand());
    //m_driverController1.rightBumper().onTrue(m_robotIntake.shootSpeakerCommand());
    //m_driverController1.leftBumper().onTrue(m_robotIntake.shootSpeakerCommand());
    //m_driverController0.rightBumper().onTrue(Commands.run(() -> Climb.Climb)));
  }
  /*public void periodic(){
    if(m_robotLimelight.getTV()){
      if(Math.sqrt(Math.pow(m_robotDrive.getPose().getX()-m_robotLimelight.getBotPose2d().getX(), 2)+Math.pow(m_robotDrive.getPose().getY()-m_robotLimelight.getBotPose2d().getY(), 2))<1){
        m_robotDrive.addVisionMeasurement(m_robotLimelight.getBotPose2d(), Timer.getFPGATimestamp());
      }
    }
  }*/
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_robotDrive.getAutonomousCommand("New Auto");
  }
  

  public void setMotorBrake(boolean brake)
  {
    m_robotDrive.setMotorBrake(brake);
  }
}