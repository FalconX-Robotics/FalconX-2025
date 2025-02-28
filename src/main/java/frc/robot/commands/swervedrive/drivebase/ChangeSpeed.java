package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ChangeSpeed extends Command{
    private SwerveSubsystem swerve;
    public ChangeSpeed(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }
    public void execute() {
        swerve.speedMode = true;
    }
    @Override
    public void end(boolean interrupted) {
        swerve.speedMode = false;
    }
} 
