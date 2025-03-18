package frc.robot.commands.swervedrive.drivebase;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveController;

public class PointToTarget extends Command {

    private SwerveSubsystem swerve;
    private final int id = 4;
    private Vision vision;
    private boolean isTurning = false;
    private double targetAngle = 0;
    private double targetYaw = 0;

    public PointToTarget(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.vision = swerve.getVision();
        addRequirements(swerve);
        isTurning = false;
    }

    public void execute() {
        Optional<Transform3d> cameraToTag = vision.getTagPose(id);
        if(cameraToTag.isPresent() && !isTurning) {
            System.out.println("Sees target");
            isTurning = true;
            System.out.println("point to target");
            targetYaw = cameraToTag.get().getRotation().getZ();
            // targetAngle = targetYaw - swerve.getYaw().getRadians();
            targetAngle = targetYaw + Math.PI;
            targetAngle = ((targetAngle + Math.PI)%(Math.PI*2)-Math.PI);
            ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(0, 0,
                                                         new Rotation2d(targetAngle));
            swerve.drive(SwerveController.getTranslation2d(desiredSpeeds), desiredSpeeds.omegaRadiansPerSecond/2, true);
        }
        // double[] tagRotation = {cameraToTag.get().getRotation().getX(),cameraToTag.get().getRotation().getY(),cameraToTag.get().getRotation().getZ()};
        // SmartDashboard.putNumberArray("tag rotation",    );
    }

    @Override
    public boolean isFinished() {
        System.out.println(Math.abs(targetAngle - swerve.getYaw().getRadians()));
        return Math.abs(Math.toDegrees(targetAngle - swerve.getYaw().getRadians())) < 10;
    }

    @Override
    public void end(boolean interrupted) {
        isTurning = false;
        System.out.println("stopped");
    }
}
