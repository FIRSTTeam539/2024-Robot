package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.Encoder;

public class Intake extends SubsystemBase{
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeSparkMaxCANID, MotorType.kBrushless);
    private final CANSparkMax shooterMotor1 = new CANSparkMax(IntakeConstants.kShooterSparkMaxCANID1, MotorType.kBrushless);
    private final CANSparkMax shooterMotor2 = new CANSparkMax(IntakeConstants.kShooterSparkMaxCANID2, MotorType.kBrushless);
    private final Encoder shooterEnc = new Encoder(IntakeConstants.kIntakeEncoderIDA, IntakeConstants.kIntakeEncoderIDB, IntakeConstants.kEncoderDirectionReversed,  IntakeConstants.kEncoderDecodingType);

    Intake(){
        intakeMotor.setIdleMode(IdleMode.kBrake);
        shooterMotor1.setIdleMode(IdleMode.kBrake);
        shooterMotor2.setIdleMode(IdleMode.kBrake);
        shooterEnc.reset();
        shooterEnc.setDistancePerPulse(IntakeConstants.kShooterDistancePerPulse/2048);

    }

    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void setShooterSpeed(double speed){
        shooterMotor1.set(speed);
        shooterMotor2.set(speed);
    }

    public Command intakeCommand(){
        return this.run(()->this.setIntakeSpeed(IntakeConstants.kIntakeSpeed));//may need to reverse direction
    }

    public Command stopIntakeCommand (){
        return this.startEnd(()-> this.setIntakeSpeed(0), ()->this.setIntakeSpeed(0));
    }

    public Command shootCommand(){
        return this.run(()->this.setShooterSpeed(IntakeConstants.kShooterSpeed));//may need to reverse direction
    }
    
    public Command stopShootCommand(){
        return this.startEnd(()->this.setShooterSpeed(0), ()->this.setShooterSpeed(0));
    }

    public double getShooterSpeed(){
        return shooterEnc.getRate();
    }
}
