// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */

  public Pigeon2 gryo;

  public Swerve() {
    // Declaring and resetting gyro to defaults
    this.gryo = new Pigeon2(9);;
  }

public static void main() {

  double wheelBase = 0; /* The distance between the centers of wheels on the same side */
  double trackWidth = 0; /* The distance between the centers of wheels on opposite sides */

  Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
  Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
  Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
  Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
  SwerveDriveOdometry odometry = new 

}

public Rotation2d getYaw() {
  double[] ypr = new double[3];
  gryo.getYawPitchRoll(ypr);
  return (false) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
