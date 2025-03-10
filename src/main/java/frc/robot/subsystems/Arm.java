// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;
import frc.robot.util.*;

public class Arm extends SubsystemBase {
  SparkMax armMotor = new SparkMax(Constants.ARM_MOTOR, MotorType.kBrushless );
  RelativeEncoder armEncoder;
  Settings settings;
  double currentSetpoint;
  PIDController pid;

  public boolean manualOverride = false;

  DoubleLogEntry angleLog = Util.createDoubleLog("arm/angle");
  DoubleLogEntry setpointLog = Util.createDoubleLog("arm/setpoint");
  DoubleLogEntry velocityLog = Util.createDoubleLog("arm/velocityLog");
  BooleanLogEntry overrideLog = Util.createBooleanLog("arm/override");

  /** Creates a new Arm. */
  public Arm( Settings settings ) {
    // Set instance variables.
    this.settings = settings;
    this.armEncoder = armMotor.getEncoder();

    // Configure the arm motor.
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    armMotorConfig.idleMode( IdleMode.kBrake );
    armMotorConfig.inverted( true );
    armMotorConfig.encoder.positionConversionFactor(Constants.ARM_CONVERSION_FACTOR);
    
    this.armMotor.configure( armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters );

    // Set the PID controller values.
    this.pid = new PIDController(4, 0, 0.2);
    pid.setSetpoint((Math.PI/2.0)-0.2);
    armMotor.getEncoder().setPosition(Math.PI/2 + Math.toRadians(5));
  }

  /**
   * Set the velocity of the motor.
   * @param velocity The velocity value of the motor. Ranges -1 (100% CCW) to +1 (100% CW)
   */
  public void setVelocity( double velocity ) {
    // Set velocity.
    armMotor.getClosedLoopController().setReference(0, ControlType.kPosition);
  }

  public void setSetpoint (double setpoint) {
    pid.setSetpoint(setpoint);
  }
  /**
   * Set the voltage of the motor.
   * @param voltage The voltage value of the motor. Ranges -12 (100% CCW) to +12 (100% CW)
   */
  public void setVoltage( double voltage ) {
    // Set voltage.
    // voltage += 1 * Math.cos(getAngle());
    this.armMotor.setVoltage( voltage );

  }

  public void setInput(double input) {
    pid.setSetpoint(pid.getSetpoint() + input * 0.01);
  }

  /**
   * Get the angle of the claw, in degrees. A zero, or perfect factor of 360, represents a forward-facing claw.
   * @return The angle of the claw.
   */
  public double getAngle() {
    
    // Return the angle.
    return armMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // if (!manualOverride) {
    //   pidController.setSetpoint(this.currentSetpoint);
    //   double pidOutput = pidController.calculate(getAngle());
    //   this.armMotor.setVoltage(pidOutput);
    // }
    if (true) {
      double pidCalc = pid.calculate(getAngle());
    
      // pidCalc = MathUtil.clamp(pidCalc, -6, 6);
      armMotor.setVoltage(pidCalc - Math.cos(getAngle()));
    }
    
    angleLog.append(getAngle());
    SmartDashboard.putNumber("Arm Angle", getAngle());
    setpointLog.append(currentSetpoint);
    velocityLog.append(armMotor.getEncoder().getVelocity());
    overrideLog.append(manualOverride);
    System.out.println("arm setpoint " + pid.getSetpoint());
    // System.out.println(getAngle());
  }
}