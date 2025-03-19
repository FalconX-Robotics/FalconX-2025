// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class MoveArm extends Command {
  Arm arm;
  CommandXboxController xboxController;


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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setInput(xboxController.getRightY() * 1 + Math.sqrt(Math.pow(0, 2)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
