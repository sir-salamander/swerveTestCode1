// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.beans.Encoder;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;

// compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html

public class Drive extends SubsystemBase {
  // Motors 
  private static PWMTalonFX LEFT_FRONT_DRIVE_SPEED_MOTOR;
  private static PWMTalonFX LEFT_BACK_DRIVE_SPEED_MOTOR;
  private static PWMTalonFX RIGHT_FRINT_DRIVE_SPEED_MOTOR;
  private static PWMTalonFX RIGHT_BACK_DRIVE_SPEED_MOTOR;

  private static PWMTalonFX LEFT_FRONT_DRIVE_DIRECTION_MOTOR;
  private static PWMTalonFX LEFT_BACK_DRIVE_DIRECTION_MOTOR;
  private static PWMTalonFX RIGHT_FRONT_DRIVE_DIRECTION_MOTOR;
  private static PWMTalonFX RIGHT_BACK_DRIVE_DIRECTION_MOTOR;

  // Encoders
  public static CANCoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
  public static CANCoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
  public static CANCoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
  public static CANCoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;

  public static CANCoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER;
  public static CANCoder LEFT_BACK_DRIVE_DRIECTION_ENCODER;
  public static CANCoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER;
  public static CANCoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER;

  // Direction encoder wrapper that scales to degrees
  public static PIDSubsystem LEFT_FRONT_DRIVE_DIRECTION_SCALED;
  public static PIDSubsystem LEFT_BACK_DRIVE_DIRECTION_SCALED;
  public static PIDSubsystem RIGHT_FRONT_DRIVE_DIRECTION_SCALED;
  public static PIDSubsystem RIGHT_BACK_DRIVE_DIRECTION_SCALED;

  // Gyro
  public static Pigeon2 gyro;

  public Drive() {
    // Motors
    LEFT_FRONT_DRIVE_SPEED_MOTOR = new PWMTalonFX(1);
    LEFT_BACK_DRIVE_SPEED_MOTOR = new PWMTalonFX(2);
    RIGHT_FRINT_DRIVE_SPEED_MOTOR = new PWMTalonFX(3);
    RIGHT_BACK_DRIVE_SPEED_MOTOR = new PWMTalonFX(4);

    LEFT_FRONT_DRIVE_DIRECTION_MOTOR = new PWMTalonFX(10);
    LEFT_BACK_DRIVE_DIRECTION_MOTOR = new PWMTalonFX(20);
    RIGHT_FRONT_DRIVE_DIRECTION_MOTOR = new PWMTalonFX(30);
    RIGHT_BACK_DRIVE_DIRECTION_MOTOR = new PWMTalonFX(40);

    // Encoders
    LEFT_FRONT_DRIVE_DISTANCE_ENCODER = new CANCoder(5);
    LEFT_BACK_DRIVE_DISTANCE_ENCODER = new CANCoder(6);
    RIGHT_FRONT_DRIVE_DISTANCE_ENCODER = new CANCoder(7);
    RIGHT_BACK_DRIVE_DISTANCE_ENCODER = new CANCoder(8);

    LEFT_FRONT_DRIVE_DIRECTION_ENCODER = new CANCoder(50);
    LEFT_BACK_DRIVE_DRIECTION_ENCODER = new CANCoder(60);
    RIGHT_FRONT_DRIVE_DIRECTION_ENCODER = new CANCoder(70);
    RIGHT_BACK_DRIVE_DIRECTION_ENCODER = new CANCoder(80);

    // Gryo
    gyro = new Pigeon2(9);
  }

  public void initDefaultCommand() {

  }
}

public class SwerveDriveWheel {
  public PIDController directionController;
  public PIDOutput directioMotor;
  public PIDSource directionSensor;

  public SwerveDriveWheel(double P, double I, double D, PIDSource directionSensor, PIDOutput directioMotor) {
    this.directionSensor = directionSensor;
    this.directioMotor = directioMotor;
    directionController = new PIDController(P, I, D, directionSensor, directioMotor);
  }

  public void setDirection(double setpoint) {
    directionController.reset();
    directionController.setSetpoint(setpoint);
    directionController.enable();
  }

  }

// Get closest angle between 2
  private static double closestAngle(double a, double b) {
    // Get direction
    double dir = modulo(b, 360.0) - modulo(a, 360.0);

    //convert from -360 to 360 to -180 to 180
    if (Math.abs(dir) > 180.0) {
      dir = -(Math.signum(dir * 360.0)) + dir;
    }
    return dir;
  }

  public void setDirection(double setpoint) {
    
  }