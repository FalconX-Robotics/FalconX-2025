package frc.robot.commands.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MoveToPosition extends SequentialCommandGroup {
    SwerveSubsystem swerve;
    Pose2d pose;
    PathConstraints constraints = new PathConstraints(2.0, 0.5, Math.toRadians(360), Math.toRadians(540));
    Command pathfindCommand;

    public MoveToPosition(SwerveSubsystem swerve, Pose2d pose) {
        this.pose = pose;
        this.swerve = swerve;
        addRequirements(swerve);
        pathfindCommand = AutoBuilder.pathfindToPose(pose, constraints, 0.0);
        addCommands(pathfindCommand);
    }
}
