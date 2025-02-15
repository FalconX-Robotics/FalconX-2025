package frc.robot.subsystems.swervedrive;

import java.nio.file.FileSystem;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    public PhotonCamera camera;
    private AprilTagFieldLayout fieldLayout;
    private boolean hasFieldLayout = false;
    
    public Vision(PhotonCamera camera) {
        this.camera = camera;
        try {
            this.fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/vision/2025-test-field.json");
        } catch (Exception e) {System.err.println("April tag layout file not found");}
    }

    public Optional<Transform3d> getTagPose(int id) {
        Optional<PhotonPipelineResult> result = getLastResult();
        if (result.isEmpty()) return Optional.empty();
        List<PhotonTrackedTarget> targets = result.get().getTargets();
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == id) {
                return Optional.of(target.getBestCameraToTarget());
            }
        }
        return Optional.empty();
    }

    public Optional<Pose2d> getFieldPose() {
        Optional<PhotonPipelineResult> result = getLastResult();
        if (result.isEmpty()) return Optional.empty();
        Optional<MultiTargetPNPResult> multiTagResult = result.get().getMultiTagResult();
        SmartDashboard.putBoolean("Has Mutitag Result", hasFieldLayout);
        if (multiTagResult.isPresent()) {
            MultiTargetPNPResult fieldResult = multiTagResult.get();
            Transform3d robotTransform = fieldResult.estimatedPose.best;
            hasFieldLayout = true;
            return Optional.of(new Pose2d(robotTransform.getX(), robotTransform.getY(), robotTransform.getRotation().toRotation2d()));

        }
        return Optional.empty();
    }
    public Optional<PhotonPipelineResult> getLastResult() {
        PhotonPipelineResult result = camera.getLatestResult();
        return Optional.ofNullable(result);
        // return camera.getLatestResult()
        // var results = camera.getAllUnreadResults();
        // if (results.size() == 0) return Optional.empty();
        // return Optional.of(results.get(results.size()-1));
    }
    public AprilTagFieldLayout getFieldLayout() {return fieldLayout;}
}
