package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.GrabCoral;
import frc.robot.commands.intake.RunIntake;

public class IntakeOff extends AutoEntry {
    public IntakeOff(String name, ArrayList<Object> params) {
        super(name, params);
    }

    public Command toCommand() {
        return new RunIntake(RobotContainer.INSTANCE.intake, 0.0);
    }
}
