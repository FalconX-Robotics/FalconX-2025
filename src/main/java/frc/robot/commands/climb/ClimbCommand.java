package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ClimbCommand extends Command {
    Settings settings;
    boolean reversed = false;
    Climb climb;
    SwerveSubsystem swerve;

    public ClimbCommand(Climb climb, SwerveSubsystem swerve, boolean reversed, Settings settings) {
        this.settings = settings;
        this.climb = climb;
        this.swerve = swerve;
        addRequirements(climb); //do not put swerve here
        this.reversed = reversed;
    }

    @Override
    public void initialize() {
        swerve.climbing = true;
       if (reversed) climb.motorRun(-settings.armSettings.climbSpeed);
       else climb.motorRun(settings.armSettings.climbSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.climbing = false;
        climb.motorRun(0);
    }
}
