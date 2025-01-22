// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArm extends Command {
  Arm arm;
  DigitalInput digitalInput = new DigitalInput( 1 );

  /** Creates a new Arm. */
  public MoveArm( Arm arm ) {
    this.arm = arm;

    addRequirements( arm ); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the mass, angle, & radius of the claw.
    final double clawMass = Constants.CLAW_MASS;
    final double clawAngle = this.arm.getAngle();
    final double clawRadius = Constants.CLAW_RADIUS;

    // Define the acceleration due to gravity.
    final double gravityAccel = -9.8;

    // Calculate the torque.
    double gravityTorque = gravityAccel * clawMass * clawRadius * Math.cos( Units.degreesToRadians( clawAngle ) );
    double magicNumber = 1;

    // Declare the velocity to use for the arm motor.
    double armMotorVelocity = gravityTorque * magicNumber;

    // TODO link an input to add/subtract motor velocity

    this.arm.setVelocity( armMotorVelocity );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
