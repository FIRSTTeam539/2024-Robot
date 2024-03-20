package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import static com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle;
import static com.revrobotics.CANSparkBase.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkBase.SoftLimitDirection.kReverse;
import static com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.controller.ArmFeedforward;



public class ArmSubsystem extends SubsystemBase{
    private final int SMART_MOTION_SLOT = 0;
      // Offset in rotations to add to encoder value - offset from arm horizontal to sensor zero
    private final CANSparkMax armLeader = new CANSparkMax(ArmConstants.kArmSparkMaxCANID1, MotorType.kBrushless);
    private final CANSparkMax armFollower = new CANSparkMax(ArmConstants.kArmSparkMaxCANID2, MotorType.kBrushless);
    //private final RelativeEncoder encoder1 = armLeader.getEncoder();
    //private final RelativeEncoder encoder2 = armFollower.getEncoder();
    private final SparkPIDController pidController = armLeader.getPIDController();
    //private final SparkPIDController m_pidController2 = armMotor2.getPIDController(); 
    private final DutyCycleEncoder armEnc = new DutyCycleEncoder(ArmConstants.kEncoderID);
    private final SparkAbsoluteEncoder armEncoder = armLeader.getAbsoluteEncoder(kDutyCycle);
    /*private final ArmFeedforward m_feedforward =
        new ArmFeedforward(
            ArmConstants.kSVolts, ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);*/

    //private Double targetPosition = null;
    /**
     * 
     */
      /** The shooter subsystem for the robot. */
    public ArmSubsystem() {
        /*super(
        new TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared),
        ArmConstants.kArmOffsetRads);*/
        //super(new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));
        /*pidController.setFeedbackDevice(armEncoder);
        pidController.setP(ArmConstants.kP);
        pidController.setI(ArmConstants.kI);
        pidController.setD(ArmConstants.kD);
        pidController.setIZone(ArmConstants.kIz);
        pidController.setFF(ArmConstants.kFF);
        pidController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

        pidController.setSmartMotionMaxVelocity(ArmConstants.maxVel, SMART_MOTION_SLOT);
        pidController.setSmartMotionMinOutputVelocity(ArmConstants.minVel, SMART_MOTION_SLOT);
        pidController.setSmartMotionMaxAccel(ArmConstants.maxAcc, SMART_MOTION_SLOT);
        pidController.setSmartMotionAllowedClosedLoopError(ArmConstants.allowedErr, SMART_MOTION_SLOT);

        // armLeader.enableVoltageCompensation(12);
        // armLeader.setSmartCurrentLimit(20);
        // armFollower.setSmartCurrentLimit(20);
        
        armLeader.setSoftLimit(kForward, ArmConstants.LIMIT_TOP);
        armLeader.setSoftLimit(kReverse, ArmConstants.LIMIT_BOTTOM);
        armLeader.enableSoftLimit(kForward, true);
        armLeader.enableSoftLimit(kReverse, true);

        // Disable limit switches, we don't have any
        armLeader.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
        armLeader.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);*/
        //armLeader.setSoftLimit(kReverse, SMART_MOTION_SLOT)
        /*MotorFeedbackSensor x = new MotorFeedbackSensor() {
            
        };*/
        //pidController.setFeedbackDevice(armEnc);
        armFollower.follow(armLeader, true);

        armLeader.setIdleMode(IdleMode.kBrake);
        armFollower.setIdleMode(IdleMode.kBrake);
        //armEncoder.setZeroOffset(0);
        //armLeader.getEncoder().setInverted(true);
        armLeader.setInverted(true);
        //armLeader.getEncoder().setPositionConversionFactor(SMART_MOTION_SLOT)

        /*// Save settings to motor flash, so they persist between power cycles
        armMotor1.burnFlash();
        armMotor2.burnFlash();*/

        //arm enc ssoudl arlready be wrrred into sparkmax so this code will not be neccecary
        armEnc.setDistancePerRotation(ArmConstants.kEncoderDistancePerRotation);
        armEnc.setPositionOffset(ArmConstants.kArmOffsetRads);
    }
    @Override
    public void periodic() {
        //SmartDashboard.putNumber("arm Position Radians", getArmPositionRadians());
        //SmartDashboard.putNumber("arm Position Raw", armEncoder.getPosition());
        //SmartDashboard.putBoolean("arm enc present", armEncoder == null);
        SmartDashboard.putNumber("raw duty cycle encoder", getRawDutyCycleEncoder());
        SmartDashboard.putNumber("radians duty cycle", getDutyCycleEncoderRadians());
        SmartDashboard.putNumber("arm possition", getArmAngleRadians());
      /*if (targetPosition != null) {
        // Calculate feed forward based on angle to counteract gravity
        double cosineScalar = Math.cos(getArmPositionRadians());
        double feedForward = ArmConstants.GRAVITY_FF * cosineScalar;
        pidController.setReference(armRadiansToEncoderRotations(targetPosition), 
            ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);
      }
  
      SmartDashboard.putNumber("arm Position Radians", getArmPositionRadians());
      SmartDashboard.putNumber("arm Position Raw", armEncoder.getPosition());*/
    }

    public void moveArmAtSpeed(double speed){
        //targetPosition = null;
        armLeader.set(speed);
        armFollower.set(-speed);
        SmartDashboard.putNumber("arm speed", speed);
        //System.out.println("this");
    }
    
    /**
    * Gets the wrist position Zero is horizontal, up is positive
    * @return position in radians
    */
    /*public double getArmPositionRadians() {
        return Units.rotationsToRadians(armEncoder.getPosition() + ArmConstants.ENCODER_OFFSET);
    }*/
    public double getRawDutyCycleEncoder(){
        return armEnc.getAbsolutePosition();
    }
    public double getDutyCycleEncoderRadians(){
        return armEnc.getAbsolutePosition()-ArmConstants.kArmOffsetRads;
    }
    public double getArmAngleRadians(){
        return armEnc.getDistance();
    }
    /*public double getArmPositionTargetRadians(){
        return targetPosition;
    }*/

    public double getArmPositionDegrees(){
        return Units.rotationsToDegrees(armEncoder.getPosition() + ArmConstants.ENCODER_OFFSET);
    }
    /*public void moveToPos(double position){
        if(position>ArmConstants.kMaxUpPos || position < ArmConstants.kMaxDownPos){
            return;
        }
        while(Math.abs(getArmAngleRadians()-position)> ArmConstants.allowedErr){
            moveArmAtSpeed(-ArmConstants.kP*(Math.abs(getArmAngleRadians()-position)));
            //armLeader.set(-MathUtil.clamp(ArmConstants.kP*(Math.abs(getArmAngleRadians()-position))+ArmConstants.holdArmPower, 
            //ArmConstants.kMaxUpSpeed, ArmConstants.kMaxDownSpeed));
            SmartDashboard.putString("test2", "yay");
        }
    }*/
    public Command moveToPosCommand(double position){
        return this.run(()->{
            //while(Math.abs(armEnc.getAbsolutePosition()-position)> ArmConstants.allowedErr){
                /*armLeader.set(-MathUtil.clamp(ArmConstants.kP*(Math.abs(armEnc.getAbsolutePosition()-position))+ArmConstants.holdArmPower, 
                ArmConstants.kMaxUpSpeed, ArmConstants.kMaxDownSpeed));*/
                this.moveArmAtSpeed(MathUtil.clamp(ArmConstants.kP*(position-getArmAngleRadians()), -0.2, 0.2));
                SmartDashboard.putNumber("goal", position);
                SmartDashboard.putNumber("error", position-getArmAngleRadians());
            //}
        }).unless(()->position>ArmConstants.kMaxUpPos || position < ArmConstants.kMaxDownPos)
        .until(()->Math.abs(getArmAngleRadians()-position)< ArmConstants.allowedErr)
        .andThen(this.run(()->this.moveArmAtSpeed(MathUtil.clamp(ArmConstants.kP*(position-getArmAngleRadians()), -0.2, 0.2))).withTimeout(0.2));
    }
    // motor.set(Clamp kp*error + ff)

    /**
    * Convert from arm position in radians to encoder rotations
    * @param armRadians arm position in radians
    * @return equivilant encoder position, in rotations
    */
    static double armRadiansToEncoderRotations(double armRadians) {
        return Units.radiansToRotations(armRadians) - ArmConstants.ENCODER_OFFSET;
    }

    /**
    * Stop the elevator
    */
    public void stop() {
        //targetPosition = null;
        armLeader.stopMotor();
    }

        /**
     *sets arm to a possition
     * 
     * @param armMoveControl control to move the arm with, + is up, - is down
     *
     * 
     */
    public void setArmVelocity(double armMoveControl){
        //targetPosition = null;
        double shootArmAxis;
        if ((getArmAngleRadians()>= ArmConstants.kMaxUpPos && armMoveControl >0)||(getArmAngleRadians()<= ArmConstants.kMaxDownPos && armMoveControl<0)){
            shootArmAxis = MathUtil.clamp(ArmConstants.holdArmPower*Math.cos(getArmAngleRadians()), ArmConstants.kMaxDownSpeed, ArmConstants.kMaxUpSpeed);
        } else if ((getArmAngleRadians()>= ArmConstants.kMaxUpPos-0.1 && armMoveControl >0.6)||(getArmAngleRadians()<= ArmConstants.kMaxDownPos+0.1 && armMoveControl<-0.1)){
            shootArmAxis = MathUtil.clamp(armMoveControl*0.25+(ArmConstants.holdArmPower*Math.cos(getArmAngleRadians())), ArmConstants.kMaxDownSpeed, ArmConstants.kMaxUpSpeed);
        } else if((getArmAngleRadians() <= Math.PI/2 && armMoveControl <0)||(getArmAngleRadians()>=Math.PI/2 &&armMoveControl>0)){
            shootArmAxis = MathUtil.clamp(armMoveControl*0.37+(ArmConstants.holdArmPower*Math.cos(getArmAngleRadians())), ArmConstants.kMaxDownSpeed, ArmConstants.kMaxUpSpeed);
        } else {
            shootArmAxis = MathUtil.clamp(armMoveControl+(ArmConstants.holdArmPower*Math.cos(getArmAngleRadians())), ArmConstants.kMaxDownSpeed, ArmConstants.kMaxUpSpeed); // Apply axis clamp and invert for driver control
        }
        armLeader.set(shootArmAxis * ArmConstants.kArmRate);
        //armFollower.set(armMoveControl);
        
        SmartDashboard.putNumber("speed", armMoveControl);
        SmartDashboard.putNumber("shoot arm move control", shootArmAxis);
        SmartDashboard.putNumber("arm power", shootArmAxis * ArmConstants.kArmRate); // put arm speed on Smartdash
        SmartDashboard.putNumber("arm vel", armLeader.getAbsoluteEncoder().getVelocity());
        SmartDashboard.putNumber("arm enc test", armLeader.getEncoder().getPosition());
        SmartDashboard.putNumber("absolute arm test", armEncoder.getVelocity());
        SmartDashboard.putNumber("absolute arm test vel", armEncoder.getPosition());
        SmartDashboard.putNumber("arm enc value", this.getArmPositionDegrees());  // put encoder value on SmartDash
    }

    public Command moveArm(double input){
        return this.run(()->this.setArmVelocity(input));
        /*if (input <= 0){
            return this.run(() ->this.moveArmAtSpeed(input * 0.6));
        } else {
            return this.run(() ->this.moveArmAtSpeed(input * 0.6));
        }*/
    }

    /*public Command setArmPos(double pos){
        return this.run(()->)
    }*/
   
    public Command disableArm () {
        return this.startEnd(() -> this.setArmVelocity(0), () -> this.setArmVelocity(0));
    }

    public Command stopArm(){
        return this.runOnce(()->this.stop());
    }

}