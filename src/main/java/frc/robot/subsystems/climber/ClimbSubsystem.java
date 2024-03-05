package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{

    private final CANSparkMax climbLeft = new CANSparkMax(ClimbConstants.kClimbSparkMaxCANIDLeft, MotorType.kBrushless);
    private final CANSparkMax climbRight = new CANSparkMax(ClimbConstants.kClimbSparkMaxCANIDRight, MotorType.kBrushless);
    private final RelativeEncoder LEncoder = climbLeft.getEncoder();
    private final RelativeEncoder REncoder = climbRight.getEncoder();


    public ClimbSubsystem(){
        climbLeft.setIdleMode(IdleMode.kBrake);
        climbRight.setIdleMode(IdleMode.kBrake);

        climbLeft.setInverted(true);

        //climbRight.follow(climbLeft, true);
        // l  307
    }

    //public double 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("left climber possition", LEncoder.getPosition());
        SmartDashboard.putNumber("right climber possition", REncoder.getPosition());
    }
    
    public void setClimb(double rate){
        climbLeft.set(-rate);
        climbRight.set(-rate);
    }
    public void setDualClimb(double lrate, double rrate){
        /*if (!(LEncoder.getPosition() <=0 && -lrate <0)){
            climbLeft.set(-lrate);
        }
        if (!(REncoder.getPosition() <= 0 && -rrate <0)){
            climbRight.set(-rrate);
        }*/
        climbLeft.set(-lrate);
        climbRight.set(-rrate);
    }
    public Command climbRightCommand(double rate){
        return this.run(()->{
            climbRight.set(-rate);
        });
    }
    public Command climbLeftCommand(double rate){
        return this.run(()->{
            climbLeft.set(-rate);
        });
    }
    public Command holdCommand(){
        return this.run(()->this.setClimb(ClimbConstants.kStaticArmRate));
    }
    public Command stop(){
        return this.startEnd(()->setClimb(0), ()->setClimb(0));
    }

    public Command climbCommand(double rate){
        /*if (rate >= ClimbConstants.kStaticArmRate && rate <= 0) {
            return this.run(()->setClimb(ClimbConstants.kStaticArmRate));
        }*/
        return this.run(()->{
            climbLeft.set(-rate);
            climbRight.set(-rate);
        });
    }
}
