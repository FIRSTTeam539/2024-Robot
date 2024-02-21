package frc.robot.commands.arm;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.Constants;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class AutoAimSpeakerCommand extends Command{
    private final LimelightSubsystem limelight;
    private final ArmSubsystem arm;
    private Command move;


    public AutoAimSpeakerCommand(ArmSubsystem arm, LimelightSubsystem limelight){
        this.limelight = limelight;
        this.arm = arm;

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
        if (limelight.getTV()){ 
            // move = arm.moveToPositionCommand(-0.00008*Math.pow(limelight.getTY(),2)+0.00252*limelight.getTY()+0.4992);
        }
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
    } else if (move.isFinished()){
            return true;
        }
        return false;
    }

}
