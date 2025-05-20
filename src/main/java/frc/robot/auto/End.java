package frc.robot.auto;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.GoToArmPosition;
import frc.robot.commands.auto.GoToArmPosition.Position;
import frc.robot.commands.intake.RunIntake;

public class End extends AutoEntry {
    public End(String name, ArrayList<Object> params) {
        super(name, params);
    }

    @Override
    public boolean safetyCheck() {
        return true;
    }

    @Override
    public Command toCommand() {
        ParallelCommandGroup commands = new ParallelCommandGroup();
        commands.addCommands(AutoBuilder.buildAuto("MFD to Start"));
        commands.addCommands(new RunIntake(RobotContainer.INSTANCE.intake, 0.0));
        commands.addCommands(new GoToArmPosition(Position.TRAVEL, RobotContainer.INSTANCE.arm, RobotContainer.INSTANCE.elevator));
        return commands;
    }
}
