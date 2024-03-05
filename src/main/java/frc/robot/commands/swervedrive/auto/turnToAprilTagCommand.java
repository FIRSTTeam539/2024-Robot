package frc.robot.commands.swervedrive.auto;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;


public class TurnToAprilTagCommand extends Command{
    private final LimelightSubsystem limelight;
    private final SwerveSubsystem swerve;

    public TurnToAprilTagCommand(SwerveSubsystem swerve, LimelightSubsystem limelight){
        this.limelight = limelight;
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if(limelight.getTV()){
            swerve.drive(new Translation2d(0, 0), (-DriveConstants.kPAprilTag*limelight.getTX()), false);
        }
        
        /*if (limelight.getTV()){
            swerve.drive(swerve.getTargetSpeeds(0 ,0, Rotation2d.fromDegrees(-limelight.getTX())));   
        }*/
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
        if (!limelight.getTV()){
            return true;
        /* else if (Math.abs(limelight.getTX())<=DriveConstants.allowedAutoAimErrorRadians){
            return true;*/
        }
        return false;
    }

}
