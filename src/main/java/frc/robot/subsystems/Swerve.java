// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;

  private limeLight LimeLight = new limeLight();
  private double getXAngle = LimeLight.getXAngle();

  private boolean trackingObject = false;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pidgeonID);
    gyro.configFactoryDefault();
    zeroGyro();

    

    mSwerveMods = new SwerveModule[] {
      new SwerveModule(0, Constants.Swerve.Mod0.constants, true),
      new SwerveModule(1, Constants.Swerve.Mod1.constants, false),
      new SwerveModule(2, Constants.Swerve.Mod2.constants, false),
      new SwerveModule(3, Constants.Swerve.Mod3.constants, true)
    };

    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean trackingObject) {
    SwerveModuleState[] swerveModuleStates = 
      Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(),
          translation.getY(),
          trackingObject ? calcultaeTrackingVelocity(rotation) : rotation,
          getYaw()
          )
          : new ChassisSpeeds(
            trackingObject ? 2.5 : translation.getX(),
            translation.getY(),
            trackingObject ? calcultaeTrackingVelocity(rotation) : rotation
          )
      );
      this.trackingObject = trackingObject;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for(SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* PathPlanner */
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
      //reset odeometry for the first path you run
      if(isFirstPath){
        this.resetOdometry(traj.getInitialHolonomicPose());
      }
    }),
    new PPSwerveControllerCommand(
      traj,
      this::getPose,
      Constants.Swerve.swerveKinematics, 
      new PIDController(0, 0, 0), 
      new PIDController(0, 0, 0), 
      new PIDController(0, 0, 0), 
      this::setModuleStates,
      true,
      this
      )
    );
  }

  /* used by swervecontrollercommand in auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for(SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void resetModulesToAbsolute() {
    for(SwerveModule mod : mSwerveMods){
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());

    for(SwerveModule mod :mSwerveMods) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

  public void setAllAngles() {
    for(SwerveModule mod : mSwerveMods) {
      mod.setAngle(new SwerveModuleState(0,new Rotation2d(0)));
    }
  }

  private double calcultaeTrackingVelocity(double rotation) {
    if (LimeLight.getXAngle() != 0 && Math.abs(LimeLight.getXAngle()) >= 0.11) {
      double speed = 1; // between 0 amd 1
      double direction = (-LimeLight.getXAngle()) / Math.abs(LimeLight.getXAngle());
      double scaleFactor = (Math.abs(LimeLight.getXAngle())) * speed;
      SmartDashboard.putNumber("tracking velocity", direction * scaleFactor);
      if (scaleFactor > 2) {
        scaleFactor = 1.4;
      }
      return -direction * scaleFactor;
    }
      
    return 0;
  }

  
}
