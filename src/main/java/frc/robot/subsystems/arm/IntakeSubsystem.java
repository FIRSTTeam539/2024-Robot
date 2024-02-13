package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.Encoder;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeSparkMaxCANID, MotorType.kBrushless);
    private final CANSparkMax shooterLeader = new CANSparkMax(IntakeConstants.kShooterSparkMaxCANID1, MotorType.kBrushless);
    private final CANSparkMax shooterFollower = new CANSparkMax(IntakeConstants.kShooterSparkMaxCANID2, MotorType.kBrushless);
    private final Encoder shooterEnc = new Encoder(IntakeConstants.kIntakeEncoderID[0], IntakeConstants.kIntakeEncoderID[1], IntakeConstants.kEncoderDirectionReversed);
    // Initializes a DigitalInput on DIO 0
    private final DigitalInput beamBreakSensor = new DigitalInput(IntakeConstants.kBeamBreakSensorId);

    public IntakeSubsystem(){
        intakeMotor.setIdleMode(IdleMode.kBrake);
        shooterLeader.setIdleMode(IdleMode.kBrake);
        shooterFollower.setIdleMode(IdleMode.kBrake);

        shooterFollower.follow(shooterLeader, false);

        shooterEnc.reset();
        shooterEnc.setDistancePerPulse(IntakeConstants.kShooterDistancePerPulse/2048);

    }
    public double getShooterSpeed(){
        return shooterEnc.getRate();
    }
    public boolean getGamePiecePresent(){
        return beamBreakSensor.get();
    }
    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void setShooterSpeed(double speed){
        shooterLeader.set(speed);
    }
    public void shootAmp(){
        this.setShooterSpeed(IntakeConstants.kShooterSpeedAmp);
    }
    public void intake(){
        //possibly switch out for if/while statement
        if (!this.getGamePiecePresent()){
            this.setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        } else{
            this.setIntakeSpeed(0);
        }
    }
    public void shootSpeaker(double speed){
        this.setShooterSpeed(speed);
        if (this.getShooterSpeed() >= speed){
            this.setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        }
    }
    public Command intakeCommand(){
        return this.run(()->this.intake());//may need to reverse direction
    }

    public Command stopIntakeCommand (){
        return this.startEnd(()-> this.setIntakeSpeed(0), ()->this.setIntakeSpeed(0));
    }

    public Command shootSpeakerCommand(){
        return this.run(()->this.shootSpeaker(IntakeConstants.kShooterSpeedSpeaker));//may need to reverse direction
    }

    public Command shootAmpCommand(){
        return this.run(()->this.shootAmp());
    }
    
    public Command stopShootCommand(){
        return this.startEnd(()->this.setShooterSpeed(0), ()->this.setShooterSpeed(0));
    }

}
