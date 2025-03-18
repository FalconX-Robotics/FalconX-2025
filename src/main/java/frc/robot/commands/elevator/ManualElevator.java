package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends Command{
    Elevator elevator;
    CommandXboxController xbox;

    public ManualElevator(Elevator elevator, CommandXboxController xbox) {
        this.elevator = elevator;
        this.xbox = xbox;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double input = xbox.getLeftY();
        input = MathUtil.applyDeadband(input, 0.1);
        elevator.setInput(input);
    }
}

