/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  private final NetworkTable m_limelightTable;
  private ArrayList<Double> m_targetList;
  private final int MAX_ENTRIES = 50;
  private final NetworkTableEntry m_isTargetValid, m_led_entry;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
    private final NetworkTableEntry tv;

//read values periodically
double x;
double y;
double area;
double v;

//post to smart dashboard periodically
//SmartDashboard.putNumber("LimelightX", x);
//SmartDashboard.putNumber("LimelightY", y);
//SmartDashboard.putNumber("LimelightArea", area);


  /**
   * Creates a new Vision.
   */
  public LimelightSubsystem() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_targetList = new ArrayList<Double>(MAX_ENTRIES);
    //m_isTargetValid = ShuffleboardInfo.getInstance().getTargetEntry();
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta  = table.getEntry("ta");
    tv  = table.getEntry("tv");

    m_isTargetValid = 
    m_led_entry = m_limelightTable.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    v = tv.getDouble(0.0);
    SmartDashboard.putNumber("Limelightx", x);
    SmartDashboard.putNumber("Limelighty", y);
    SmartDashboard.putNumber("LimelightArea", area);

    m_isTargetValid.setBoolean(isTargetValid());

    if (m_targetList.size() >= MAX_ENTRIES) {
      m_targetList.remove(0);
    }
    m_targetList.add(area);
  }

  public double getTX() {
    return x;
  }

  public double getTA() {
    double sum = 0;

    for (Double num : m_targetList) { 		      
      sum += num.doubleValue();
    }
    return sum/m_targetList.size();
  }

  public boolean isTargetValid() {
    return (v == 1.0); 
  }

  public void setLlLedMode(int mode){
    m_led_entry.setDouble((mode));
  }
}