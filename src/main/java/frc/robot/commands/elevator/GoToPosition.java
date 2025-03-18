package frc.robot.commands.elevator;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class GoToPosition extends Command {

    public static enum Level {
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        LEVEL_4
    }

    Level level;
    Elevator elevator;

    HashMap<Level, Double> levels = new HashMap<>();
    
    public GoToPosition(Level level, Elevator elevator) {
        this.level = level;
        this.elevator = elevator;
        addRequirements(elevator);

        levels.put(Level.LEVEL_1, 18.0);
        levels.put(Level.LEVEL_2, 31.875);
        levels.put(Level.LEVEL_3, 47.625);
        levels.put(Level.LEVEL_4, 72.0);
    }
    
    @Override
    public void initialize() {
        elevator.setSetpoint(levels.get(level));
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setInput(0);
    }

}
