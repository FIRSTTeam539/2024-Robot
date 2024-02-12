/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Optional;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
//import frc.robot.LimelightHelpers.LimelightResultsWrapper;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
//import frc.robot.LimelightHelpers.LimelightTarget_Retro;


/**
 * LimelightSubsystem
 */
public class LimelightSubsystem extends SubsystemBase {

  private LimelightResults latestLimelightResults = null;

  private final NetworkTable limelightNetworkTable;
  private final String networkTableName;

  private boolean takeSnapshot = false;

  private boolean enabled;
  private boolean driverMode;
  private double activePipelineId;
  private ObjectMapper mapper;

  public LimelightSubsystem(String networkTableName) {
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable(networkTableName);
    this.networkTableName = networkTableName;

    limelightNetworkTable.getEntry("snapshot").setDouble(0.0);

    new Trigger(RobotState::isEnabled)
        .onTrue(Commands.runOnce(this::enable))
        .onFalse(Commands.runOnce(this::disable, this).ignoringDisable(true));
  }

  /**
   * Parses Limelight's JSON results dump into a LimelightResults Object
   */
  public LimelightResults getLatestResults() {
    if (latestLimelightResults == null) {
      if (mapper == null) {
        mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
      }

      try {
        var json = limelightNetworkTable.getEntry("json").getString("");
        var wrapper = mapper.readValue(json, LimelightResults.class);
        latestLimelightResults = LimelightHelpers.getLatestResults(networkTableName);
      } catch (JsonProcessingException e) {
        System.err.println("lljson error: " + e.getMessage());
      }
    }
    return latestLimelightResults;
  }
  
  /*public Optional<LimelightTarget_Retro> getLatestRetroTarget() {
    var results = getLatestResults();
    if (results != null && results.targetingResults.valid && results.targetingResults.RetroreflectiveTargets.length > 0) {
      return Optional.of(results.RetroreflectiveTargets[0]);
    }
    return Optional.empty();
  }*/

  @Override
  public void periodic() {
    latestLimelightResults = null;
    // Flush NetworkTable to send LED mode and pipeline updates immediately
    var shouldFlush = (limelightNetworkTable.getEntry("ledMode").getDouble(0.0) != (enabled ? 0.0 : 1.0) || 
        limelightNetworkTable.getEntry("pipeline").getDouble(0.0) != activePipelineId);
    limelightNetworkTable.getEntry("ledMode").setDouble(enabled ? 0.0 : 1.0);
    limelightNetworkTable.getEntry("camMode").setDouble(driverMode ? 1.0 : 0.0);
    limelightNetworkTable.getEntry("pipeline").setDouble(activePipelineId);
  
    if (shouldFlush)  {
      NetworkTableInstance.getDefault().flush();
    }

    if(takeSnapshot) {
      limelightNetworkTable.getEntry("snapshot").setDouble(1.0);
      takeSnapshot = false;
    } else {
      limelightNetworkTable.getEntry("snapshot").setDouble(0.0);
    }
  }

  public Pose2d getBotPose2d(){
    return LimelightHelpers.getBotPose2d(networkTableName);
  }
  public Pose3d getBotPose3d(){
    return LimelightHelpers.getBotPose3d(networkTableName);
  }

  /**
   * Turns the LEDS off and switches the camera mode to vision processor.
   */
  public void disable() {
    enabled = false;
    driverMode = false;
  }

  /**
   * Sets the LEDS to be controlled by the pipeline and switches the camera mode
   * to vision processor.
   */
  public void enable() {
    enabled = true;
    driverMode = false;
  }

  /**
   * Sets the LEDs to off and switches the camera to driver mode.
   */
  public void driverMode() {
    enabled = false;
    driverMode = true;
  }

  public String getNetworkTableName() {
    return networkTableName;
  }

  public void takeSnapshot() {
    takeSnapshot = true;
  }

  public void setPipelineId(int pipelineId) {
    activePipelineId = pipelineId;
  }

}


// package frc.robot.subsystems;

// import java.security.Key;
// import java.util.ArrayList;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.LimelightHelpers;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

// public class LimelightSubsystem extends SubsystemBase {
//   private final NetworkTable m_limelightTable;
//   private ArrayList<Double> m_targetList;
//   private final int MAX_ENTRIES = 50;
//   private final NetworkTableEntry m_isTargetValid, m_led_entry;
//   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
//   private final NetworkTableEntry tx;
//   private final NetworkTableEntry ty;
//   private final NetworkTableEntry ta;
//   private final NetworkTableEntry tv;
//   private final NetworkTableEntry tl;
//   private final NetworkTableEntry botpose;
//   private final NetworkTableEntry botpose_wpiblue;
//   private final NetworkTableEntry botpose_wpired;
//   private final NetworkTableEntry camerapose_targetspace;
//   private final NetworkTableEntry targetpose_cameraspace;
//   private final NetworkTableEntry targetpose_robotspace;
//   private final NetworkTableEntry botpose_targetspace;
//   private final NetworkTableEntry camerapose_robotspace;
//   private final NetworkTableEntry tid;
  

// //read values periodically
// double x;
// double y;
// double area;
// double v;

// //post to smart dashboard periodically
// //SmartDashboard.putNumber("LimelightX", x);
// //SmartDashboard.putNumber("LimelightY", y);
// //SmartDashboard.putNumber("LimelightArea", area);


//   /**
//    * Creates a new Vision.
//    */
//   public LimelightSubsystem() {
//     m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
//     m_targetList = new ArrayList<Double>(MAX_ENTRIES);
//     //m_isTargetValid = ShuffleboardInfo.getInstance().getTargetEntry();
//     tx = table.getEntry("tx");
//     ty = table.getEntry("ty");
//     ta  = table.getEntry("ta");
//     tv  = table.getEntry("tv");
//     tl = table.getEntry("tl");
//     botpose = table.getEntry("botpose");
//     botpose_wpiblue = table.getEntry("botpose_wpiblue");
//     botpose_wpired = table.getEntry("botpose_wpired");
//     camerapose_targetspace = table.getEntry("camerapose_targetspace");
//     targetpose_cameraspace = table.getEntry("targetpose_cameraspace");
//     targetpose_robotspace = table.getEntry("targetpose_robotspace");
//     botpose_targetspace = table.getEntry("botpose_targetspace");
//     camerapose_robotspace = table.getEntry("camerapose_robotspace");
//     tid = table.getEntry("tid");


//     m_isTargetValid = 
//     m_led_entry = m_limelightTable.getEntry("ledMode");
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     x = tx.getDouble(0.0);
//     y = ty.getDouble(0.0);
//     area = ta.getDouble(0.0);
//     v = tv.getDouble(0.0);
//     SmartDashboard.putNumber("Limelightx", x);
//     SmartDashboard.putNumber("Limelighty", y);
//     SmartDashboard.putNumber("LimelightArea", area);

//     m_isTargetValid.setBoolean(isTargetValid());

//     if (m_targetList.size() >= MAX_ENTRIES) {
//       m_targetList.remove(0);
//     }
//     m_targetList.add(area);
//   }

//   public double getTX() {
//     return x;
//   }

//   public double getTA() {
//     double sum = 0;

//     for (Double num : m_targetList) { 		      
//       sum += num.doubleValue();
//     }
//     return sum/m_targetList.size();
//   }

//   public boolean isTargetValid() {
//     return (v == 1.0); 
//   }

//   public void setLlLedMode(int mode){
//     m_led_entry.setDouble((mode));
//   }
//   public double getTL(){
//     return tl.getDouble(0.00);
//   }
//   /*public double[] getBotPose(){
//     return botpose.getDoubleArray(null);
//   }*/

// }