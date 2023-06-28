// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  /** Creates a new SwerveModuleConstants. */

public final int driveMotorID;
public final int angleMotorID;
public final int cancoderID;
public final Rotation2d angleOffset;

  public SwerveModuleConstants(int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = cancoderID;
    this.angleOffset = angleOffset;
  }
}
