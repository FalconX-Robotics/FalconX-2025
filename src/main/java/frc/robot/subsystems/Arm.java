// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.dyn4j.dynamics.Settings;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  SparkMax armMotor = new SparkMax( 25, MotorType.kBrushless );
  RelativeEncoder armEncoder;
  Settings settings;

  /** Creates a new Arm. */
  public Arm( Settings settings ) {
    // Set instance variables.
    this.settings = settings;
    this.armEncoder = armMotor.getEncoder();

    // Configure the arm motor.
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode( IdleMode.kBrake );

    this.armMotor.configure( armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters );
  }

  /**
   * Set the velocity of the motor.
   * @param velocity The velocity value of the motor. Ranges -1 (100% CCW) to +1 (100% CW)
   */
  public void setVelocity( double velocity ) {
    // Set velocity.
    this.armMotor.set( velocity );
  }

  /**
   * Get the angle of the claw, in degrees. A zero, or perfect factor of 360, represents a forward-facing claw.
   * @return The angle of the claw.
   */
  public double getAngle() {
    // Get the # of rotations that the claw has made. Then calculate the total by 360 (rotations -> degrees).
    SparkAbsoluteEncoder absoluteEncoder = this.armMotor.getAbsoluteEncoder();
    double totalRotations = absoluteEncoder.getPosition();
    double angleDegrees = totalRotations * 360;

    // Return the angle.
    return angleDegrees;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}