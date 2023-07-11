// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;

public class limeLight extends SubsystemBase {
  //NetworkTable fields
  NetworkTable table;
  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;  
  NetworkTableEntry tid;
  NetworkTableEntry getpipe;
  NetworkTableEntry pipeline;

  //Other fields
  RobotContainer m_robotContainer;
  XboxController m_controller;
  Command m_teleopCommand;

  //Initialize fields and get NetworkTable for the first limelight
  public limeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tid = table.getEntry("tid");
    getpipe = table.getEntry("getpipe");
    pipeline = table.getEntry("pipeline");
  }

  public void setTeleopCommand(Command teleopCommand) {
    m_teleopCommand = teleopCommand;
  }

  //Returns the horizontal angle the detected object 
  public double getXAngle() {
    return tx.getDouble(0.0);
  }

  public double getYAngle() {
    return ty.getDouble(0.0);
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }

  public double getTagID() {
    return tid.getDouble(0.0);
  }

  public double getPipline() {
    return getpipe.getDouble(0.0);
  }

  public boolean hasTarget() {
    if (tv.getDouble(0.0) == 1) {
      return true;
    } else {
      return false;
    }
  }

  /** 
   * Set pipline number between 0-9.
  */
  public void setPipeline(double pipelineNumber) {
    pipeline.setValue(pipelineNumber);
  }

  public void setTargetedAprilTagId(double targetedAprilTagId) {
    setPipeline(targetedAprilTagId + 1);
  }

  public double calculateDistance(double targetHeight) {
    return 0;
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("LimelightX", getXAngle());
    SmartDashboard.putNumber("LimelightY", getYAngle());
    SmartDashboard.putNumber("LimelightArea", getArea());
    SmartDashboard.putNumber("CurrentTargetedTagID", getTagID());
    SmartDashboard.putNumber("CurrentPipline", getPipline());
  }

  @Override
  public void periodic() {
    
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}