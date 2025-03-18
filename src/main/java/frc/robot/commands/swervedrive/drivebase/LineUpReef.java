package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.util.Util;

public class LineUpReef extends Command {
    public static enum Side {
        LEFT,
        RIGHT
    }
    // private static enum State {
    //     DRIVE,
    //     CORRECT
    // }

    private SwerveSubsystem swerve;
    private Vision vision;
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
        vision = swerve.getVision();
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
            goToTarget = swerve.driveToPose(targetPose).andThen(getAdjustCommand(id, side));
            goToTarget.schedule();
            SmartDashboard.putNumber("Line Up Reef/Pose X", targetPose.getX());
            SmartDashboard.putNumber("Line Up Reef/Pose Y", targetPose.getY());
            SmartDashboard.putNumber("Line Up Reef/Pose Angle", targetPose.getRotation().getDegrees());
        } else {
            throw new RuntimeException("LineUpReef Target does not exist");
        }
    }
    
    Command getAdjustCommand(int index, Side side) {
        return Commands.run(()-> {
            Pose2d currentOffset = Util.transformToPose(vision.getTagPose(index).get()).toPose2d();
            switch (side) {
                case LEFT:
                    currentOffset = new Pose2d(currentOffset.getX() - 0.5, currentOffset.getY() + 0.5, currentOffset.getRotation());
                    break;
                case RIGHT:
                    currentOffset = new Pose2d(currentOffset.getX() - 0.5, currentOffset.getY() - 0.5, currentOffset.getRotation());
                    break;
            }
            double xSpeed = -currentOffset.getX() / 3;
            double ySpeed = -currentOffset.getY() / 3;
            double rotSpeed = -currentOffset.getRotation().getRadians()/3;
            Translation2d translation = new Translation2d(xSpeed, ySpeed);
            swerve.drive(translation, rotSpeed, false);
        }, swerve).until(()-> {
            return Util.transformToPose(vision.getTagPose(index).get()).toPose2d().relativeTo(Pose2d.kZero).getTranslation().getNorm() < 0.1;
        });
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