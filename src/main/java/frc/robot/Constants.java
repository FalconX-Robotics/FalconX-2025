// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Util;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = Util.poundsToKilos(126.6); // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(10);
  public static final double MAX_ANGULAR_VELOCITY = 1 * Math.PI;
  public static final double MAX_ANGULAR_ACCELERATION = 1 * Math.PI;
  public static final double CLAW_MASS = 0;
  public static final double CLAW_SPEED = 3;

  public static final Pose3d ROBOT_TO_CAMERA_POSE = new Pose3d(0, 0, 0.5, new Rotation3d());

  public static final Pose2d START_POSE = new Pose2d(1.427, 2.259, Rotation2d.fromDegrees(0));

  public static final double MAX_VISION_AMBIGUITY = 0.25;
      // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final double CLAW_RADIUS = 0;
  public static final double MAX_INTAKE_SPEED = 1.0;
  public static final double MAX_RELEASE_SPEED = -1.0;
  public static final double MAX_ANGLE_UP_SPEED = 1.0;
  public static final double MAX_ANGLE_DOWN_SPEED = -1.0;

  public static final int CORAL_SENSOR = 1;
  public static final int ARM_MOTOR = 20;
  public static final int FEEDER_MOTOR = 21;
  public static final int INTAKE_MOTOR = 22;
  public static final int ELEVATOR_MOTOR = 30;
  public static final int CLIMB_ID = 40;

  public static final int LED_PORT = 1;
  public static final int ELEVATOR_HIGH_SWITCH = 5;
  public static final int ELEVATOR_LOW_SWITCH = 0;

  public static final double ARM_CONVERSION_FACTOR = (1.0/(6.21430317958)) * Math.PI/2;

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
    
  }

  public static final class DrivebaseConstants
  {
    public static final double SPEED_MODE_SCALE = 0.2;
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    // public static final double SPEED_MODE_SCALE = 0.5;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ID 
  {

    public static final int armID = 22;


  }

public static class GearRatio 
{
  //change when we know the REAL gear ratio
  public static final double armGearRatio = 0.5;


}

}
/**
 * PDH 1
 * BR Drive 10
 * BL Angle 17
 * BL Drive 16
 * FL Angle 15
 * FL Drive 14
 * BR Angle 11
 * FR Angle 13
 * FR Drive 12
 */