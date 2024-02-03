package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
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
import static com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle;
import static com.revrobotics.CANSparkBase.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkBase.SoftLimitDirection.kReverse;
import static com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj2.command.Command;
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
    private static final double ENCODER_OFFSET = -0.58342d;
    private static final double GRAVITY_FF = 0.01;
    private static final float LIMIT_BOTTOM = 0.5804f;
    private static final float LIMIT_TOP = 0.8995f;
    private final CANSparkMax armLeader = new CANSparkMax(ArmConstants.kArmSparkMaxCANID1, MotorType.kBrushless);
    private final CANSparkMax armFollower = new CANSparkMax(ArmConstants.kArmSparkMaxCANID2, MotorType.kBrushless);
    private final RelativeEncoder encoder1 = armLeader.getEncoder();
    private final RelativeEncoder encoder2 = armFollower.getEncoder();
    private final SparkPIDController pidController = armLeader.getPIDController();
    //private final SparkPIDController m_pidController2 = armMotor2.getPIDController(); 
    private final DutyCycleEncoder armEnc = new DutyCycleEncoder(ArmConstants.kEncoderID);
    private final SparkAbsoluteEncoder armEncoder = armLeader.getAbsoluteEncoder(kDutyCycle);
    private final ArmFeedforward m_feedforward =
        new ArmFeedforward(
            ArmConstants.kSVolts, ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    private Double targetPosition = null;
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
        pidController.setFeedbackDevice(armEncoder);
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

        armLeader.enableVoltageCompensation(12);
        armLeader.setSmartCurrentLimit(20);
        armFollower.setSmartCurrentLimit(20);
        
        armLeader.setSoftLimit(kForward, LIMIT_TOP);
        armLeader.setSoftLimit(kReverse, LIMIT_BOTTOM);
        armLeader.enableSoftLimit(kForward, true);
        armLeader.enableSoftLimit(kReverse, true);

        // Disable limit switches, we don't have any
        armLeader.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
        armLeader.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);

        armFollower.follow(armLeader, true);

        armLeader.setIdleMode(IdleMode.kBrake);
        armFollower.setIdleMode(IdleMode.kBrake);

        /*// Save settings to motor flash, so they persist between power cycles
        armMotor1.burnFlash();
        armMotor2.burnFlash();*/

        //arm enc ssoudl arlready be wrrred into sparkmax so this code will not be neccecary
        armEnc.setDistancePerRotation(ArmConstants.kEncoderDistancePerRotation);
        armEnc.setPositionOffset(ArmConstants.kArmOffsetRads);
    }
    @Override
    public void periodic() {
      if (targetPosition != null) {
        // Calculate feed forward based on angle to counteract gravity
        double cosineScalar = Math.cos(getArmPosition());
        double feedForward = GRAVITY_FF * cosineScalar;
        pidController.setReference(armRadiansToEncoderRotations(targetPosition), 
            ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);
      }
  
      SmartDashboard.putNumber("arm Position Radians", getArmPosition());
      SmartDashboard.putNumber("arm Position Raw", armEncoder.getPosition());
    }

    public void moveArmAtSpeed(double speed){
        targetPosition = null;
        armLeader.set(speed);
    }

    /**
   * Moves the elevator to a position. Zero is horizontal, up is positive. There is no motor safety, so calling this
   * will continue to move to this position, and hold it until another method is called.
   * @param radians position in radians
   */
    public void moveToPosition(double radians) {
        // Set the target position, but move in execute() so feed forward keeps updating
        targetPosition = radians;
    }

    public Command moveToPositionCommand(double radians){
        return this.run(()->this.moveToPosition(radians));
    }

    /**
    * Gets the wrist position Zero is horizontal, up is positive
    * @return position in radians
    */
    public double getArmPosition() {
        if (targetPosition != null) {
            // Calculate feed forward based on angle to counteract gravity
            double cosineScalar = Math.cos(getArmPosition());
            double feedForward = GRAVITY_FF * cosineScalar;
            pidController.setReference(armRadiansToEncoderRotations(targetPosition), 
            ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);
      }
      {

      }
        return Units.rotationsToRadians(armEncoder.getPosition() + ENCODER_OFFSET);
    }

    /**
    * Convert from arm position in radians to encoder rotations
    * @param armRadians arm position in radians
    * @return equivilant encoder position, in rotations
    */
    static double armRadiansToEncoderRotations(double armRadians) {
        return Units.radiansToRotations(armRadians) - ENCODER_OFFSET;
    }

    /**
    * Stop the elevator
    */
    public void stop() {
        targetPosition = null;
        armLeader.stopMotor();
    }
  
    /*
    @Override
    protected void useState(TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        pidController.setReference(6, CANSparkMax.ControlType.kPosition);
        //m_pidController2.setReference(6, CANSparkMax.ControlType.kPosition);
        armMotor1.setSetpoint(
            .PIDMode.kPosition, setpoint.position, feedforward / 12.0)
        
    }*/
   /* ArmSubsystem (){
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);
        armEnc.reset();

    }*/
    /*
    @Override
    public void useOutput(double output, double setpoint) {
        armMotor1.set(setpoint);
        //m_ArmMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
    }
    @Override
    public double getMeasurement() {
      return armEnc.getDistance();
    }*/
  

        /**
     *sets arm to a possition
     * 
     * @param armMoveControl control to move the arm with, + is up, - is down
     *
     * 
     */
    private void setArmVelocity(double armMoveControl){

        double shootArmAxis = -MathUtil.clamp(armMoveControl, ArmConstants.kMaxDownSpeed, ArmConstants.kMaxUpSpeed); // Apply axis clamp and invert for driver control
        armLeader.set(shootArmAxis * ArmConstants.kArmRate);
        
        SmartDashboard.putNumber("arm power", shootArmAxis * ArmConstants.kArmRate); // put arm speed on Smartdash
        SmartDashboard.putNumber("arm enc value", armEncoder.getPosition());  // put encoder value on SmartDash
    }

    public Command moveArm(double input){
        return this.run(() ->this.setArmVelocity(input));
    }

    /*public Command setArmPos(double pos){
        return this.run(()->)
    }*/
   
    public Command disableArm () {
        return this.startEnd(() -> this.setArmVelocity(0), () -> this.setArmVelocity(0));
    }

    public double getAbsolutePossition(){
        return armEnc.getAbsolutePosition();
    }

    public void resetEncoder(){
        armEnc.reset();
    }
}