package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.arm.IntakeSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.swervedrive.auto.TurnToAprilTagCommand;
import frc.robot.commands.arm.AutoAimSpeakerCommand;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class AutoShootSpeakerCommand extends Command{
    private final LimelightSubsystem limelight;
    private final SwerveSubsystem swerve;
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;
    private final TurnToAprilTagCommand turn;
    private final AutoAimSpeakerCommand aim;

    AutoShootSpeakerCommand(SwerveSubsystem swerve, LimelightSubsystem limelight, ArmSubsystem arm, IntakeSubsystem intake){
        this.swerve = swerve;
        this.limelight = limelight;
        this.intake = intake;
        this.arm = arm;

        aim = new AutoAimSpeakerCommand(arm, limelight);
        turn = new TurnToAprilTagCommand(swerve, limelight);

        addRequirements(swerve);
        addRequirements(intake);
        addRequirements(arm);
    }
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        /*Commands.sequence(
            Commands.parallel(
                turn,
                aim
            ).andThen(intake.shootSpeakerCommand())
        );*/
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        //add condition if done
        return false;
    }
}
