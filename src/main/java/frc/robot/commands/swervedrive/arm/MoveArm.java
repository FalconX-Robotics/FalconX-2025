// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArm extends Command {
  Arm arm;
  CommandXboxController xboxController;

  boolean usingPID = false;

  /** 
   * Creates a new Arm.
   * @param arm The arm to move
   * @param controller The Xbox controller to link the arm to.
   */
  public MoveArm( Arm arm, CommandXboxController controller ) {
    // Set instance variables.
    this.arm = arm;
    this.xboxController = controller;
    this.addRequirements( arm ); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.manualOverride = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // boolean startButtonPressed = this.xboxController.start().getAsBoolean();

    // if ( startButtonPressed ) usingPID = !usingPID;
    //   // Define the angles by the buttons.
    //   final double aButtonAngle = 20;
    //   final double bButtonAngle = 45;
    //   final double xButtonAngle = 60;

    //   // Get the current states of the buttons.
    //   final boolean aButtonPressed = this.xboxController.a().getAsBoolean();
    //   final boolean bButtonPressed = this.xboxController.b().getAsBoolean();
    //   final boolean xButtonPressed = this.xboxController.x().getAsBoolean();

    //   // Set the setpoint of the PID controller.
    //   if ( aButtonPressed ) this.arm.setSetpoint(aButtonAngle);
    //   if ( bButtonPressed ) this.arm.setSetpoint(bButtonAngle);
    //   if ( xButtonPressed ) this.arm.setSetpoint(xButtonAngle);

    arm.setVoltage(xboxController.getRightY()  * 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setVoltage(0);
    arm.manualOverride = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
