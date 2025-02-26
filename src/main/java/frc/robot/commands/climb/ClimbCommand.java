package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommand extends Command {
    Settings settings;
    boolean reversed = false;
    Climb climb;
    public ClimbCommand(Climb climb, boolean reversed, Settings settings) {
        this.settings = settings;
        this.climb = climb;
        addRequirements(climb);
        this.reversed = reversed;
    }

    @Override
    public void initialize() {
       if (reversed == true) climb.motorRun(-settings.armSettings.climbSpeed);
       else climb.motorRun(settings.armSettings.climbSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        climb.motorRun(0);
    }
}
