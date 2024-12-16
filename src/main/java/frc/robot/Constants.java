// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.jni.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.parser.PIDFConfig;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (125) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.5; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    //public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    //public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    //public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);


    
    //public static final PIDFConfig TranslationPID = new PIDFConfig(0, 0, 0);
    //public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;
  }
   public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.1, 0, 0.0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }


  public static final class ArmConstants{
    public static final int kArmSparkMaxCANID1 = 15; // reconfigure to actual values
    public static final int kArmSparkMaxCANID2 = 16;//set to value
    public static final int kEncoderID = 0; //adjust to actual value

    public static final double kSVolts = 0; //smallest value to make arm move miscule amount - static movment amount
    public static final double kGVolts = 0;
    public static final double kVVoltSecondPerRad =0;
    public static final double kAVoltSecondSquaredPerRad = 0;

    public static final double kArmOffsetRads = 0.08;

    public static final double kP = 0.8;


    /*
    public static final double kI = 1e-6;
    public static final double kD = 0;
    public static final double kF = 0;
    public static final double kIz = 0;
    public static final double kFF = 1.00;*/

    public static final double holdArmPower = 0.085;


    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final double maxRPM = 5700;
    public static final double maxVel = 2000;
    public static final double minVel = -2000;
    public static final double maxAcc = 1500;
    public static final double kMaxVelocityRadPerSecond =10; //set latter
    public static final double kMaxAccelerationRadPerSecSquared = 275;
    public static final double allowedErr= Units.degreesToRadians(5);//change

    public static final double ENCODER_OFFSET = 0.08d;
    //public static final double GRAVITY_FF = 0.01;
    public static final float LIMIT_BOTTOM = 0f;
    public static final float LIMIT_TOP = (float) (2*Math.PI);

    public static final double kEncoderDistancePerRotation = 2*Math.PI; // give us 2 rad per rotation


    public static final double kArmRate = 0.5;

    //min/ max value of arm
    public static final double kMaxDownSpeed= -0.5;
    public static final double kMaxUpSpeed= 0.5;
    public static final double kMaxUpPos = 1.82;
    public static final double kMaxDownPos = 0.004;
    //set value of deadzone


  //move arm power
  public static final double ARM_OUTPUT_POWER = 0.7;
  } 

  public static final class IntakeConstants{
    public static final int kIntakeSparkMaxCANID = 17; //change to actual value
    public static final int kShooterSparkMaxCANID1 = 18; 
    public static final int kShooterSparkMaxCANID2 = 19;

    //public static final int[] kIntakeEncoderID = {1,2};
    public static final boolean kEncoderDirectionReversed = false;
    public static final  CounterBase.EncodingType kEncoderDecodingType = Encoder.EncodingType.k4X; 
    public static final double shooterWheelRadius = Units.inchesToMeters(2); //in meter per seconds
    public static final double kShooterDistancePerPulse = 2*Math.PI*shooterWheelRadius; // in meters

    public static int kBeamBreakSensorId = 2;

    public static final double kIntakeSpeed = 0.3;
    public static final double kShooterSpeedSpeaker = 0.95;
    public static final double kShooterSpeedAmp = 0.6;
    //public static final double kMaxShooterSpeedMetersPerSecond = 0.5;

  }

  public static final class ClimbConstants {
    public static final int kClimbSparkMaxCANIDLeft = 13; // change to real value
    public static final int kClimbSparkMaxCANIDRight = 14; // change to real value

    public static final double kStaticArmRate = 0;
  }

  public static final class SHOOTER_LIMELIGHT {
    public static final String NAME = "limelight-shooter";

    public static final int HUMAN_PIPELINE_INDEX = 0;

    public static final int GAMEPIECE_INDEX = 2;
    public static final int AMP_PIPELINE_INDEX = 3;
    public static final int SPEAKER_PIPELINE_INDEX = 4; // ids 4 and 7

    public static final boolean IS_PRIMARY_STREAM = false;

    public static final double MOUNTING_ANGLE_DEGREES = 0.0;
    public static final double MOUNTING_HEIGHT_INCHES = 0.0;

    public static final double DEFAULT_RETURN_VALUE = 0.0;

    public static final double HEAD_ON_TOLERANCE = 0.0;

    public static final double TARGET_WIDTH = 0.0;
    public static final double TARGET_HEIGHT = 0.0;

    public static final double TARGET_HEIGHT_FROM_FLOOR = 0.0;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double WHEEL_LOCK_TIME = 10;

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);

    //public static final double allowedAutoAimErrorRadians = Units.degreesToRadians(0.1);



    //drive to abriltag
    public static final PIDController APRILTAG_ROTATION_PID_CONTROLLER =
        new PIDController(3, 0, 0.01);
    public static final PIDController APRILTAG_X_TRANSLATION_PID_CONTROLLER =
        new PIDController(0.05, 0, 0);
    public static final PIDController APRILTAG_Y_TRANSLATION_PID_CONTROLLER =
        new PIDController(0.2, 0, 0);

    public static final double APRILTAG_X_TOLERANCE = 1.5;
    public static final double APRILTAG_Y_TOLERANCE = 0.5;
    public static final double APRILTAG_ROTATION_TOLERANCE = .025; // Radians
    public static final double APRILTAG_TY_MAGIC_OFFSET = 12.5;

    public static final double AMP_TX_SETPOINT = 0;
    public static final double AMP_TY_SETPOINT = -10;
    public static final double AMP_ROTATION_SETPOINT = Math.PI / 2;
    public static final double AUTO_TRANSLATE_DEBOUNCE_SECONDS = 0.1;

    public static final double kPAprilTag = 0.3;

  }
  public static final class LimelightConstants{
    public static final double forward = 0;
    public static final double side = 0;
    public static final double up = 0;
    public static final double pitch = 0;
    public static final double roll = 0;
    public static final double yaw = 0; 
  }

  /*public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }*/

  /**
   * 
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort0 = 0;
    public static final int kDriverControllerPort1 = 1;
    public static final double kDriveDeadband = 0.05;


    public static final double kDefaultDriveSpeed = 0.8;
    public static final double kDriveSpeedIncreaseConstant = 1- kDefaultDriveSpeed;
    //controller 0
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND_1 = 0.05;
    public static final double LEFT_Y_DEADBAND_1 = 0.05;
    public static final double RIGHT_X_DEADBAND_1 = 0.05;

    public static final double RIGHT_TRIGGER_DEADBAND_1 = 0.05;
    public static final double TURN_CONSTANT = 0.75;

    //controller 1
    public static final double RIGHT__Y_DEADBAND_2 = 0.05;
    public static final double LEFT_X__DEADBAND_2 = 0.1;
    public static final double RIGHT_TRIGGER_DEADBAND_2 = 0.05;
  }



}