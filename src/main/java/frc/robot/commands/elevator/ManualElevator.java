package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.XboxController;
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
    public void initialize() {
        elevator.override =true;
    }

    @Override
    public void execute() {
        
        elevator.setVelocity(xbox.getLeftY()/2);
    }
    @Override
    public void end(boolean interrupted) {
        elevator.override = false;
    }
}

