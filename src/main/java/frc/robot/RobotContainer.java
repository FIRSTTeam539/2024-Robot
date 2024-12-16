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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.auto.TurnToAprilTagCommand;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.NamedCommands;

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
  //private final ArmSubsystem m_robotArm = new ArmSubsystem();
  //private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  //private final ClimbSubsystem m_robotClimb = new ClimbSubsystem();
  //private final LimelightSubsystem m_robotLimelight = new LimelightSubsystem("limelight");
    
  SendableChooser<Command> m_chooser = new SendableChooser<>();
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
    //register pathplanner named commands
    /*NamedCommands.registerCommand("shoot", m_robotIntake.shootSpeakerCommand());
    NamedCommands.registerCommand("intake", m_robotIntake.intakeCommand());
    NamedCommands.registerCommand("intake for 1 seconds", m_robotIntake.intakeCommand().withTimeout(1));
    NamedCommands.registerCommand("move arm to shoot speaker at sub", m_robotArm.moveToPosCommand(0.47).withTimeout(4));
    NamedCommands.registerCommand("move arm to shoot from side", m_robotArm.moveToPosCommand(0.43).withTimeout(4));
    NamedCommands.registerCommand("move arm to intake", m_robotArm.moveToPosCommand(0.03));*/

    configureButtonBindings();
    // Configure default commands
    
    TeleopDrive teleopDrive = new TeleopDrive(m_robotDrive, 
      (()->-MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.LEFT_Y_DEADBAND_1)*(OIConstants.kDefaultDriveSpeed+
      OIConstants.kDriveSpeedIncreaseConstant*MathUtil.applyDeadband(m_driverController0.getRightTriggerAxis(), OIConstants.RIGHT_TRIGGER_DEADBAND_1))), 
      (()->-MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.LEFT_X_DEADBAND_1)*(OIConstants.kDefaultDriveSpeed+
      OIConstants.kDriveSpeedIncreaseConstant*MathUtil.applyDeadband(m_driverController0.getRightTriggerAxis(), OIConstants.RIGHT_TRIGGER_DEADBAND_1))),  
      (()->-MathUtil.applyDeadband(
      m_driverController0.getRightX(), OIConstants.RIGHT_X_DEADBAND_1)*(OIConstants.kDefaultDriveSpeed+
      OIConstants.kDriveSpeedIncreaseConstant*m_driverController0.getRightTriggerAxis())), 
      ()->true);
        AbsoluteDrive absoluteDrive = new AbsoluteDrive(m_robotDrive, 
      ()->MathUtil.applyDeadband(-m_driverController0.getLeftY(), 0.1)*0.7,
      ()->MathUtil.applyDeadband(-m_driverController0.getLeftX(), 0.1)*0.7,
      ()->MathUtil.applyDeadband(-m_driverController0.getRightY(), 0.1)*0.7,
      ()->MathUtil.applyDeadband(-m_driverController0.getRightX(), 0.1)*0.7);
    /*TeleopDrive simClosedFieldRel = new TeleopDrive(m_robotDrive,
      (()->MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.LEFT_Y_DEADBAND_1)), 
      (()->MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.LEFT_X_DEADBAND_1)), 
      (()->MathUtil.applyDeadband(m_driverController0.getRightX(), OIConstants.LEFT_X_DEADBAND_1)), 
      ()->true);*/
  
    //m_robotDrive.setDefaultCommand(!RobotBase.isSimulation() ? teleopDrive : simClosedFieldRel);
    m_robotDrive.setDefaultCommand(teleopDrive);
   /* m_robotClimb.setDefaultCommand(Commands.run(()->m_robotClimb.setDualClimb(MathUtil.applyDeadband(m_driverController1.getLeftY(), 0.2), MathUtil.applyDeadband(m_driverController1.getRightY(), 0.2)), m_robotClimb)); // works
    m_robotArm.setDefaultCommand(Commands.run(()->m_robotArm.setArmVelocity(MathUtil.applyDeadband(m_driverController1.getRightTriggerAxis(), 0.1)-MathUtil.applyDeadband(m_driverController1.getLeftTriggerAxis(),0.1)),m_robotArm));

    m_robotIntake.setDefaultCommand(Commands.run(()->m_robotIntake.disable(), m_robotIntake));*/
    
    //Shuffleboard.getTab("Arm").add(m_robotArm);

    Shuffleboard.getTab("Important").add("auto chooser", m_chooser);
    // m_chooser.setDefaultOption("Just Shoot", justScoreAuto);
    // m_chooser.addOption("shoot 1 and drive", moveAndJustScoreAuto);
    // m_chooser.addOption("test", testAuto);
    // m_chooser.addOption("test2 electric bugaloo", test2Auto);
    // m_chooser.addOption("test 3", test3);
    m_chooser.addOption("Just Shoot", new PathPlannerAuto("Just Shoot"));
    m_chooser.addOption("Amp Side 2 Note", new PathPlannerAuto("Amp Side 2 Note"));
    m_chooser.addOption("Amp Side Wait", new PathPlannerAuto("Amp Side Wait"));
    m_chooser.addOption("Center 2 Note", new PathPlannerAuto("Center 2 Note"));
    m_chooser.addOption("Center 3 Note Long", new PathPlannerAuto("Center 3 Note Long"));
    m_chooser.addOption("Center 2 Note and Taxi", new PathPlannerAuto("Center 2 Note and Taxi"));
    m_chooser.addOption("Center 3 Note", new PathPlannerAuto("Center 3 Note"));
    m_chooser.addOption("Stage Side 2 Note", new PathPlannerAuto("Stage Side 2 Note"));
    m_chooser.addOption("Stage Side 2.5 Note", new PathPlannerAuto("Stage Side 2.5 Note"));
    m_chooser.addOption("Stage Side Taxi 2 Note", new PathPlannerAuto("Stage Side Taxi 2 Note"));
    m_chooser.addOption("Stage Side Taxi 2 Note New", new PathPlannerAuto("Stage Side Taxi 2 Note New"));
    m_chooser.addOption("Stage Side 3 Note", new PathPlannerAuto("Stage Side 3 Note"));
    m_chooser.addOption("do nothing", null);
    m_chooser.addOption("1m test", new PathPlannerAuto("1m test"));
    m_chooser.addOption("test Auto", new PathPlannerAuto("test Auto"));
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
    /*m_driverController0.leftBumper().whileTrue(m_robotArm.moveToPosCommand(0.1));
    m_driverController0.rightBumper().whileTrue(m_robotArm.moveToPosCommand(Math.PI/3));
    
    //m_driverController1.leftBumper().whileTrue(Commands.run(()->m_robotClimb.setDualClimb(MathUtil.applyDeadband(m_driverController1.getLeftY(), 0.2), MathUtil.applyDeadband(m_driverController1.getRightY(), 0.2)*0.6), m_robotClimb));
    m_driverController1.rightBumper().whileTrue(m_robotIntake.shootSpeakerCommand());
    m_driverController1.y().whileTrue(m_robotIntake.intakeCommand().andThen(m_robotArm.moveToPosCommand(0.1)));
    m_driverController1.x().whileTrue(m_robotIntake.shootAmpCommand());
    //m_driverController1.a().onTrue(m_robotArm.disableArm());
    m_driverController1.povUp().whileTrue(Commands.run(()->m_robotArm.setArmVelocity(MathUtil.applyDeadband(m_driverController1.getRightTriggerAxis(), 0.1)-MathUtil.applyDeadband(m_driverController1.getLeftTriggerAxis(),0.1)*0.25), m_robotArm));
    
    //m_driverController1.a().whileTrue(m_robotIntake.justShootCommand(1));//MathUtil.applyDeadband(m_driverController1.getLeftTriggerAxis(),0.1)*0.5));
    m_driverController1.b().whileTrue(m_robotIntake.justIntakeCommand(0.3 ));
    m_driverController1.povDown().whileTrue(m_robotIntake.justIntakeCommand(-0.2));
    m_driverController1.povRight().whileTrue(m_robotArm.moveToPosCommand(0.450));
    //m_driverController1.povRight().whileTrue(m_robotArm.moveToPosCommand(0.450));
    //m_driverController1.povRight().whileTrue(m_robotArm.moveToPosCommand(0.3919));
    m_driverController1.povLeft().whileTrue(m_robotArm.moveToPosCommand(1.067));
    //m_driverController1.povLeft().whileTrue(m_robotArm.moveToPosCommand(0.6128));
    
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
    //m_driverController0.rightBumper().onTrue(Commands.run(() -> Climb.Climb)));*/
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
    return m_chooser.getSelected();
    /*return m_robotArm.moveToPosCommand(Math.PI/4)
      .andThen(m_robotArm.moveToPosCommand(0.3919))
      .andThen(m_robotIntake.shootSpeakerCommand());*/
    //return new SequentialCommandGroup(m_robotArm.moveToPosCommand(Math.PI/3), m_robotArm.moveToPosCommand(0.3919), m_robotIntake.shootAmpCommand());
    //return m_robotDrive.getAutonomousCommand("test Auto");
  }
  

  public void setMotorBrake(boolean brake)
  {
    m_robotDrive.setMotorBrake(brake);
  }
}