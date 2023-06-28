// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double stickDeadband = 0.1;

  public static final class Swerve {
    public static final int pidgeonID = 1;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = 
      COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L1);

    public static final double trackWidth = Units.inchesToMeters(21.73);
    public static final double wheelBase = Units.inchesToMeters(21.73);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics */
    public static final SwerveDriveKinematics swerveKinmatics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    /* module gear ratio */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    static final double angleGearRatio = chosenModule.angleGearRatio;

    /* motor inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* angle encoder invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve current limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 25;
    public static final int drivePeakCurrentLimit = 35;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* helps with treadwear, etc*/
    public static final double openloopRamp = .25;
    public static final double closedloopRamp = 0.0;

    /* Angle motor PID */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* drive motor PID */
    public static final double driveKP = 0.05;
    public static final double driveKI = 0.0;
    public static final double driveKD = 12.0;
    public static final double driveKF = 0.0;

    /* drive motor charachterization values */
    public static final double driveKS = (.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    public static double maxSpeed = 1.5; // mps = 3.96
    public static double maxAngularVelocity = 5;

    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
          public static final int driveMotorID = 6;
          public static final int angleMotorID = 5;
          public static final int canCoderID = 11;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(177.45);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module - Module 1 */
      public static final class Mod1 {
          public static final int driveMotorID = 7;
          public static final int angleMotorID = 8;
          public static final int canCoderID = 12;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(145.107);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
      
      /* Back Left Module - Module 2 */
      public static final class Mod2 {
          public static final int driveMotorID = 3;
          public static final int angleMotorID = 4;
          public static final int canCoderID = 9;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(285.47);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module - Module 3 */
      public static final class Mod3 {
          public static final int driveMotorID = 2;
          public static final int angleMotorID = 1;
          public static final int canCoderID = 10;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(258.574);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  }

  public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;

      /* constraint for the motion profile */
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
