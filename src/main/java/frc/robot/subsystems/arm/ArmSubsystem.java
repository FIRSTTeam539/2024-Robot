package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;



public class ArmSubsystem extends PIDSubsystem{
    private final CANSparkMax armMotor1 = new CANSparkMax(ArmConstants.kArmSparkMaxCANID1, MotorType.kBrushless);
    private final CANSparkMax armMotor2 = new CANSparkMax(ArmConstants.kArmSparkMaxCANID2, MotorType.kBrushless);
    private final DutyCycleEncoder armEnc = new DutyCycleEncoder(ArmConstants.kEncoderID);
    private final ArmFeedforward m_shooterFeedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts, ArmConstants.kVVoltSecondsPerRad);
    /**
     * 
     */
      /** The shooter subsystem for the robot. */
    ArmSubsystem() {
        super(new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));
        armEnc.setDistancePerRotation(ArmConstants.kEncoderDistancePerRotation);
        armEnc.setPositionOffset(ArmConstants.kArmOffsetRads);
    }
   /* ArmSubsystem (){
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);
        armEnc.reset();

    }*/
    @Override
    public void useOutput(double output, double setpoint) {
        armMotor1.set(setpoint);
        //m_ArmMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
    }
    @Override
    public double getMeasurement() {
      return armEnc.getDistance();
    }
  

        /**
     *sets arm to a possition
     * 
     * @param armMoveControl control to move the arm with, + is up, - is down
     *
     * 
     */
    private void setArmVelocity(double armMoveControl){

        double shootArmAxis = -MathUtil.clamp(armMoveControl, ArmConstants.kMaxDownSpeed, ArmConstants.kMaxUpSpeed); // Apply axis clamp and invert for driver control
        armMotor1.set(shootArmAxis * -ArmConstants.kArmRate);
        armMotor2.set(shootArmAxis * ArmConstants.kArmRate);
        
        SmartDashboard.putNumber("arm power", shootArmAxis * ArmConstants.kArmRate); // put arm speed on Smartdash
        SmartDashboard.putNumber("arm1 enc value", armMotor2.getEncoder().getPosition()); 
        SmartDashboard.putNumber("arm2 enc value", armMotor2.getEncoder().getPosition()); // put encoder value on SmartDash
    }

    public Command moveArm(double input){
        return this.run(() ->this.setArmVelocity(input));
    }

    /**
     * Current does not work; will set up as command latter set up as PID subsystem
     * @param goalPos
     */
    public void setArmPos(double goalPos){
        if(goalPos > this.getAbsolutePossition()){
            setArmVelocity(ArmConstants.kMaxUpSpeed);
        } else if (goalPos < this.getAbsolutePossition()){
            setArmVelocity(ArmConstants.kMaxDownSpeed);
        }
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