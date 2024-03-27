package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeSparkMaxCANID, MotorType.kBrushless);
    private final CANSparkMax shooterLeader = new CANSparkMax(IntakeConstants.kShooterSparkMaxCANID1, MotorType.kBrushless);
    private final CANSparkMax shooterFollower = new CANSparkMax(IntakeConstants.kShooterSparkMaxCANID2, MotorType.kBrushless);
    //private final SparkAbsoluteEncoder shooterncoder = shooterLeader.getAbsoluteEncoder(kDutyCycle);
    //private final Encoder shooterEnc = new Encoder(IntakeConstants.kIntakeEncoderID[0], IntakeConstants.kIntakeEncoderID[1], IntakeConstants.kEncoderDirectionReversed);
    // Initializes a DigitalInput on DIO 1
    private final DigitalInput beamBreakSensor = new DigitalInput(IntakeConstants.kBeamBreakSensorId);

    public IntakeSubsystem(){
        intakeMotor.setIdleMode(IdleMode.kBrake);
        shooterLeader.setIdleMode(IdleMode.kBrake);
        shooterFollower.setIdleMode(IdleMode.kBrake);

        intakeMotor.setInverted(true);

        shooterFollower.follow(shooterLeader, false);

        //shooterEnc.reset();
        //shooterEnc.setDistancePerPulse(IntakeConstants.kShooterDistancePerPulse/2048);

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("game piece", getGamePiecePresent());
        SmartDashboard.putBoolean("beam break test",beamBreakSensor.get());
        // TODO Auto-generated method stub
        //super.periodic();
        //SmartDashboard.putNumber("shooter encoder", getShooterSpeed());
    }

    /*public double getShooterSpeed(){
        return shooterEnc.getRate();
    }*/
    public boolean getGamePiecePresent(){
        return !beamBreakSensor.get();
    }
    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void setShooterSpeed(double speed){
        shooterLeader.set(speed);
    }
    public void shootAndIntake(double speed){
        intakeMotor.set(IntakeConstants.kIntakeSpeed);
        shooterLeader.set(speed);
    }
    public void shootAmp(){
        this.setShooterSpeed(IntakeConstants.kShooterSpeedAmp);
    }
    public void intake(){
        //possibly switch out for if/while statement
        this.setIntakeSpeed(IntakeConstants.kIntakeSpeed) ;
        /*if (!this.getGamePiecePresent()){
            this.setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        } else{
            this.setIntakeSpeed(0);
        }*/
    }
    public void shootSpeaker(double speed){
        this.setShooterSpeed(speed);
        /*if (this.getShooterSpeed() >= speed){
            this.setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        }*/
    }
    public Command intakeCommand(){
        return this.run(()->this.intake())
        .finallyDo(()->this.stopIntake());
       //.andThen(this.run(()->this.setIntakeSpeed(-0.2)).withTimeout(0.05));//may need to reverse direction
    }
    public void stopShoot(){
        this.setShooterSpeed(0);
    }
    public void stopIntake(){
        this.setIntakeSpeed(0);
    }  
    public Command stopIntakeCommand (){
        return this.startEnd(()-> this.setIntakeSpeed(0), ()->this.setIntakeSpeed(0));
    }
    public Command shootSpeakerCommand(){
        //Timer time = new Timer();
        //time.reset();
        //time.restart();
        //time.start();
        return this.run(()->this.setIntakeSpeed(-0.2)).withTimeout(0.05)
        .andThen(this.run(()->{
            this.setShooterSpeed(IntakeConstants.kShooterSpeedSpeaker);
        }).withTimeout(0.3))//.until(()->time.get()>=0.2)//.withTimeout(0.1)//.until(()->time.get()>=0.2)
        /*.andThen(this.run(()->{
            //time.stop();
            this.shootAndIntake(IntakeConstants.kShooterSpeedSpeaker);}).until(()->!this.getGamePiecePresent()).withTimeout(0.5))
        */.andThen(this.run(()->{
            this.shootAndIntake(IntakeConstants.kShooterSpeedSpeaker);}).withTimeout(0.5))
        .andThen(this.run(()->{
            this.stopIntake();
            this.stopShoot();
        }).withTimeout(0.1));//.withTimeout(2);//.withTimeout(2);
        /* .until(()->!this.getGamePiecePresent())
        .andThen(()->this.shootAndIntake(IntakeConstants.kShooterSpeedSpeaker)).withTimeout(0.05);*/
    }

    /*
    public Command shootSpeakerCommand(){
        return this.run(()->this.shootSpeaker(IntakeConstants.kShooterSpeedSpeaker));//may need to reverse direction
    }*/
    /*public Command shootAtAmpCommand(){
        return this.run(()-> this.shootAndIntake(IntakeConstants.kShooterSpeedAmp))
        .until(()->!this.getGamePiecePresent())
        .andThen(()->this.shootAndIntake(IntakeConstants.kShooterSpeedAmp)).withTimeout(0.1);
    }*/
    public Command shootAmpCommand(){
        return this.run(()->{
            this.setShooterSpeed(IntakeConstants.kShooterSpeedAmp);
            //SmartDashboard.putNumber("timer", time.get());
        }).withTimeout(0.2)//.until(()->time.get()>=0.2)//.withTimeout(0.1)//.until(()->time.get()>=0.2)
        .andThen(this.run(()->{
            this.shootAndIntake(IntakeConstants.kShooterSpeedAmp);}).withTimeout(1))
        .finallyDo(()->{
            this.stopIntake();
            this.stopShoot();
            });//.withTimeout(2);//.withTimeout(2);
        /* .until(()->!this.getGamePiecePresent())
        .andThen(()->this.shootAndIntake(IntakeConstants.kShooterSpeedSpeaker)).withTimeout(0.05);*/
    }
    public void disable(){
        shooterLeader.set(0);
        intakeMotor.set(0);
    }
    
    public Command stopShootCommand(){
        return this.startEnd(()->this.setShooterSpeed(0), ()->this.setShooterSpeed(0));
    }
    public Command justIntakeCommand(double speed){
        return this.run(()->intakeMotor.set(speed));//.withInterruptBehavior(this.startEnd(()->this.stopIntake(), ()->this.stopIntake()));
    }
    public Command justShootCommand(double speed){
        return this.run(()->{shooterLeader.set(speed);
        shooterFollower.set(speed);});
    }

}
