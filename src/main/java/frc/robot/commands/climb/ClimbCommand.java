package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ClimbCommand extends Command {
    Settings settings;
    boolean reversed = false;
    Climber climb;
    SwerveSubsystem swerve;

    public ClimbCommand(Climber climb, SwerveSubsystem swerve, boolean reversed, Settings settings) {
        this.settings = settings;
        this.climb = climb;
        this.swerve = swerve;
        // the swerve likely is unnecessary in addRequirements();
        // this unschedules the swerve command even though you are already turning off odometry updates
        // you should probably just set all the swerve motors to 0 in SwerveSubsystem.java -w
        addRequirements(climb, swerve);
        this.reversed = reversed;
    }

    @Override
    public void initialize() {
        swerve.climbing = true;
        if (reversed) climb.motorRun(-settings.armSettings.climbSpeed);
        else          climb.motorRun(settings.armSettings.climbSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.climbing = false;
        climb.motorRun(0);
    }
}
