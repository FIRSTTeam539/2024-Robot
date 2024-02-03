package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ClimbConstants;

public class Climb{

    private final CANSparkMax climbMotor1 = new CANSparkMax(ClimbConstants.kClimbSparkMaxCANID1, MotorType.kBrushless);
    private final CANSparkMax climbMotor2 = new CANSparkMax(ClimbConstants.kClimbSparkMaxCANID1, MotorType.kBrushless);

    Climb(){
        climbMotor1.setIdleMode(IdleMode.kBrake);
        climbMotor2.setIdleMode(IdleMode.kBrake);
    }
    
    public void setClimb(double rate){
        climbMotor1.set(rate);
        climbMotor2.set(rate);
    }
}
