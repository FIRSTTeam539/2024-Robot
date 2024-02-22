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
import frc.robot.Constants.LimelightConstants;;
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
    LimelightHelpers.setCameraPose_RobotSpace(networkTableName, LimelightConstants.forward, LimelightConstants.side,
      LimelightConstants.up, LimelightConstants.roll, LimelightConstants.pitch, LimelightConstants.yaw);

    new Trigger(RobotState::isEnabled)
        .onTrue(Commands.runOnce(this::enable))
        .onFalse(Commands.runOnce(this::disable, this).ignoringDisable(true));
  }

  /**
   * Parses Limelight's JSON results dump into a LimelightResults Object
   */
  public LimelightResults getLatestResults() {
    /*if (latestLimelightResults == null) {
      if (mapper == null) {
        mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
      }*/

      //try {
        //var json = limelightNetworkTable.getEntry("json").getString("");
        //var wrapper = mapper.readValue(json, LimelightResults.class);
        latestLimelightResults = LimelightHelpers.getLatestResults(networkTableName);
      /*} catch (JsonProcessingException e) {
        System.err.println("lljson error: " + e.getMessage());
      }*/
    //}
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
  public Pose2d getBotPose2d_wpiRed(){
    return LimelightHelpers.getBotPose2d_wpiRed(networkTableName);
  }
  public Pose2d getBotPose2d_wpiBlue(){
    return LimelightHelpers.getBotPose2d_wpiBlue(networkTableName);
  }
  public Pose3d getBotPose3d(){
    return LimelightHelpers.getBotPose3d(networkTableName);
  }
  public Pose3d getBotPose3d_wpiRed(){
    return LimelightHelpers.getBotPose3d_wpiRed(networkTableName);
  }
  public Pose3d getBotPose3d_wpiBlue(){
    return LimelightHelpers.getBotPose3d_wpiBlue(networkTableName);
  }
  public Pose3d getBotPose3d_TargetSpace(){
    return LimelightHelpers.getBotPose3d_TargetSpace(networkTableName);
  }
  public Pose3d getCameraPose3d_RobotSpace(){
    return LimelightHelpers.getCameraPose3d_RobotSpace(networkTableName);
  }
  public Pose3d gettargetaPose3d_RobotSpace(){
    return LimelightHelpers.getTargetPose3d_RobotSpace(networkTableName);
  }
  public double getTX(){
    return  LimelightHelpers.getTX(networkTableName);
  }
  public boolean getTV(){
    return LimelightHelpers.getTV(networkTableName);
  }

  public double getTY(){
    return LimelightHelpers.getTY(networkTableName);
  }
  public double getTA(){
    return LimelightHelpers.getTA(networkTableName);
  }
  public double getAprilTagID(){
    return LimelightHelpers.getFiducialID(networkTableName);
  }
  public double getLatency(){
    return LimelightHelpers.getLatency_Pipeline(networkTableName)+LimelightHelpers.getLatency_Capture(networkTableName);
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