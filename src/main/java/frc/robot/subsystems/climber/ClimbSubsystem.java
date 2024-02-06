package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{

    private final CANSparkMax climbLeader = new CANSparkMax(ClimbConstants.kClimbSparkMaxCANID1, MotorType.kBrushless);
    private final CANSparkMax climbFollower = new CANSparkMax(ClimbConstants.kClimbSparkMaxCANID1, MotorType.kBrushless);

    public ClimbSubsystem(){
        climbLeader.setIdleMode(IdleMode.kBrake);
        climbFollower.setIdleMode(IdleMode.kBrake);

        climbFollower.follow(climbLeader);
    }
    
    public void setClimb(double rate){
        climbLeader.set(rate);
    }
    public Command holdCommand(){
        return this.run(()->this.setClimb(ClimbConstants.kStaticArmRate));
    }

    public Command climbCommand(double rate){
        if(rate>= ClimbConstants.kStaticArmRate){
            return this.run(()->setClimb(rate));
        } else{
            return this.run(()->setClimb(ClimbConstants.kStaticArmRate));
        }
    }
}
