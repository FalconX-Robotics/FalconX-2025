// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.*;

public class Arm extends SubsystemBase {
  SparkMax armMotor = new SparkMax(Constants.ARM_MOTOR, MotorType.kBrushless );
  RelativeEncoder armEncoder;
  double currentSetpoint;
  PIDController pid = new PIDController(5, 0, 0.4);
  ArmFeedforward feedforward = new ArmFeedforward(0, 0.17, 2.50, 0.02);

  public boolean manualOverride = false;

  public final double ARM_OFFSET = 0.0; // In degrees

  DoubleLogEntry angleLog = Util.createDoubleLog("arm/angle");
  DoubleLogEntry setpointLog = Util.createDoubleLog("arm/setpoint");
  DoubleLogEntry velocityLog = Util.createDoubleLog("arm/velocityLog");
  BooleanLogEntry overrideLog = Util.createBooleanLog("arm/override");

  /** Creates a new Arm. */
  public Arm() {
    // Set instance variables.
    this.armEncoder = armMotor.getEncoder();

    // Configure the arm motor.
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode( IdleMode.kBrake );
    armMotorConfig.inverted( true );
    armMotorConfig.encoder.positionConversionFactor(Constants.ARM_CONVERSION_FACTOR);
    
    this.armMotor.configure( armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters );

    // Set the PID controller values.
    pid.setSetpoint((Math.PI/2.0)-0.2);
    armMotor.getEncoder().setPosition(Math.PI/2 + Math.toRadians(5));
  }

  public void setSetpoint (double setpoint) {
    pid.setSetpoint(setpoint);
  }
  /**
   * Set the voltage of the motor.
   * @param voltage The voltage value of the motor. Ranges -12 (100% CCW) to +12 (100% CW)
   */
  public void setVoltage( double voltage ) {
    this.armMotor.setVoltage( voltage );

  }

  public void setInput(double input) {
    input = MathUtil.applyDeadband(input, 0.1);
    input *= 1.8 * 1.8 * 1.8 * 1.8;
    pid.setSetpoint(pid.getSetpoint() + input * 0.01);
  }

  /**
   * Get the angle of the claw, in degrees. A zero, or perfect factor of 360, represents a forward-facing claw.
   * @return The angle of the claw.
   */
  public double getAngle() {
    
    // Return the angle.
    return armMotor.getEncoder().getPosition() - ARM_OFFSET;
  }

  @Override
  public void periodic() {
    double pidCalc = pid.calculate(getAngle());
    SmartDashboard.putNumber("arm/PID Output", pidCalc);
    double ffCalc = feedforward.calculate(pid.getSetpoint(), pidCalc);
    SmartDashboard.putNumber("arm/FF Output", ffCalc);

    pidCalc = MathUtil.clamp(pidCalc, -4, 4);

    // armMotor.setVoltage(pidCalc - Math.cos(getAngle()));
    armMotor.setVoltage(pidCalc);
    
    angleLog.append(getAngle());
    SmartDashboard.putNumber("Arm Angle", getAngle());
    setpointLog.append(pid.getSetpoint());
    velocityLog.append(armMotor.getEncoder().getVelocity());
    overrideLog.append(manualOverride);
    // System.out.println(getAngle());
  }
}