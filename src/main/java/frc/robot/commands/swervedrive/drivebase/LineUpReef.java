package frc.robot.commands.swervedrive.drivebase;

import static edu.wpi.first.units.Units.Rotation;

import java.io.Serializable;
import java.nio.channels.ScatteringByteChannel;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LineUpReef extends Command {
    public static enum Side {
        LEFT,
        RIGHT
    }

    private SwerveSubsystem swerve;
    private int id;
    private Pose2d tagPose;
    private Pose2d targetPose;
    private Side side;
    private AprilTagFieldLayout field;
    private Command goToTarget;

    public LineUpReef(SwerveSubsystem swerve, int id, Side side) {
        this.swerve = swerve;
        this.id = id;
        this.side = side;
        field = swerve.getVision().getFieldLayout();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Line Up Reef/Running", true);
        if (field.getTagPose(id).isPresent()) {
            tagPose = field.getTagPose(id).get().toPose2d();
            double x = tagPose.getX() + tagPose.getRotation().getCos();
            double y = tagPose.getY() + tagPose.getRotation().getSin();
            Rotation2d rot = Rotation2d.fromDegrees(tagPose.getRotation().getDegrees() + 180);
            targetPose = new Pose2d(x, y, rot);
            goToTarget = swerve.driveToPose(targetPose);
            goToTarget.schedule();
            SmartDashboard.putNumber("Line Up Reef/Pose X", targetPose.getX());
            SmartDashboard.putNumber("Line Up Reef/Pose Y", targetPose.getY());
            SmartDashboard.putNumber("Line Up Reef/Pose Angle", targetPose.getRotation().getDegrees());
        } else {
            throw new RuntimeException("LineUpReef Target does not exist");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        SmartDashboard.putBoolean("Line Up Reef/Running", false);
    }
    @Override
    public boolean isFinished() {
        System.out.println(goToTarget.isFinished());
        return goToTarget.isFinished();
    }
}