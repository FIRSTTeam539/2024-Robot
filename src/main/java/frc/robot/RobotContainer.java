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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  // The driver's controller
  CommandXboxController m_driverController0 = new CommandXboxController(OIConstants.kDriverControllerPort0);
  CommandXboxController m_driverController1 = new CommandXboxController(OIConstants.kDriverControllerPort1);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Configure default commands
    TeleopDrive teleopDrive = new TeleopDrive(m_robotDrive, 
      (()->MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.LEFT_Y_DEADBAND)*(OIConstants.kDefaultDriveSpeed+
      OIConstants.kDriveSpeedIncreaseConstant*MathUtil.applyDeadband(m_driverController0.getRightTriggerAxis(), OIConstants.RIGHT_TRIGGER_DEADBAND))), 
      (()->MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.LEFT_X_DEADBAND)*(OIConstants.kDefaultDriveSpeed+
      OIConstants.kDriveSpeedIncreaseConstant*MathUtil.applyDeadband(m_driverController0.getRightTriggerAxis(), OIConstants.RIGHT_TRIGGER_DEADBAND))),  
      (()->MathUtil.applyDeadband(-m_driverController0.getRightX(), OIConstants.RIGHT_X_DEADBAND)*(OIConstants.kDefaultDriveSpeed+
      OIConstants.kDriveSpeedIncreaseConstant*m_driverController0.getRightTriggerAxis())), 
      ()->true);
    
    TeleopDrive simClosedFieldRel = new TeleopDrive(m_robotDrive,
      (()->MathUtil.applyDeadband(m_driverController0.getLeftY(), OIConstants.LEFT_Y_DEADBAND)), 
      (()->MathUtil.applyDeadband(m_driverController0.getLeftX(), OIConstants.LEFT_X_DEADBAND)), 
      (()->MathUtil.applyDeadband(m_driverController0.getRightX(), OIConstants.LEFT_X_DEADBAND)), 
      ()->true);
  
    m_robotDrive.setDefaultCommand(!RobotBase.isSimulation() ? teleopDrive : simClosedFieldRel);
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
    m_driverController0.y().onTrue(Commands.run(() -> m_robotDrive.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_robotDrive.getAutonomousCommand("New Path", true);
  }
  

  public void setMotorBrake(boolean brake)
  {
    m_robotDrive.setMotorBrake(brake);
  }
}