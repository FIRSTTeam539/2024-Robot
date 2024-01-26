package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb {

    private final CANSparkMax climbMotor1 = new CANSparkMax(ClimbConstants.kClimbSparkMaxCANID1, MotorType.kBrushless);
    private final CANSparkMax climbMotor2 = new CANSparkMax(ClimbConstants.kClimbSparkMaxCANID1, MotorType.kBrushless);

    Climb(){
        climbMotor1.setIdleMode(IdleMode.kBrake);
        climbMotor2.setIdleMode(IdleMode.kBrake);
    }
    
}