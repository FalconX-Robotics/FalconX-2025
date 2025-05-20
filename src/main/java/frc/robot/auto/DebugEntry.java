package frc.robot.auto;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.RunIntake;

public class DebugEntry extends AutoEntry {

    public DebugEntry(String name, ArrayList<Object> params) throws Exception {
        super(name, params);
    }

    public Command toCommand() {
        return new PrintCommand(parameters.toString());
    }
}
