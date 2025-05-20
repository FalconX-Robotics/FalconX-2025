package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ResetAutoPos extends Command {
    SwerveSubsystem swerve;
    Pose2d pose;

    public ResetAutoPos(Pose2d pose) {
        this.swerve = RobotContainer.INSTANCE.swerve;
        this.pose = pose;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.resetOdometry(pose);
    }
}
