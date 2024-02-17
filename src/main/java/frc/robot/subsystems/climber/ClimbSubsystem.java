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

    private final CANSparkMax climbLeft = new CANSparkMax(ClimbConstants.kClimbSparkMaxCANIDLeft, MotorType.kBrushless);
    private final CANSparkMax climbRight = new CANSparkMax(ClimbConstants.kClimbSparkMaxCANIDLeft, MotorType.kBrushless);

    public ClimbSubsystem(){
        climbLeft.setIdleMode(IdleMode.kBrake);
        climbRight.setIdleMode(IdleMode.kBrake);

        //climbRight.follow(climbLeft, true);
    }
    
    public void setClimb(double rate){
        climbLeft.set(rate);
        climbRight.set(rate);
    }
    public Command climbRightCommand(double rate){
        return this.run(()->{
            climbRight.set(rate);
        });
    }
    public Command climbLeftCommand(double rate){
        return this.run(()->{
            climbLeft.set(rate);
        });
    }
    public Command holdCommand(){
        return this.run(()->this.setClimb(ClimbConstants.kStaticArmRate));
    }

    public Command climbCommand(double rate){
        if (rate >= ClimbConstants.kStaticArmRate && rate <= 0) {
            return this.run(()->setClimb(ClimbConstants.kStaticArmRate));
        }
        return this.run(()->setClimb(rate));
    }
}
