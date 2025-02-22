package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommand extends Command {
    double voltage = -1.8;
    boolean reversed = false;
    Climb climb;
    public ClimbCommand(Climb climb, boolean reversed) {
        this.climb = climb;
        addRequirements(climb);
        this.reversed = reversed;
    }

    @Override
    public void initialize() {
       if (reversed == true) climb.motorRun(-voltage);
       else climb.motorRun(-voltage);
    }
}
