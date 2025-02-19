package frc.robot.subsystems.swervedrive;

import java.nio.file.FileSystem;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision {
    public PhotonCamera camera;
    private AprilTagFieldLayout fieldLayout;
    private boolean hasFieldLayout = false;

    private VisionSystemSim visionSim = new VisionSystemSim("Limelight");
    private TargetModel aprilTag = TargetModel.kAprilTag36h11;
    private SimCameraProperties cameraSimProperties = new SimCameraProperties();
    private PhotonCameraSim cameraSim;
    private SwerveSubsystem swerve;
    
    public Vision(PhotonCamera camera, SwerveSubsystem swerve) {
        this.camera = camera;
        this.swerve = swerve;
        try {
            if (Robot.isReal()) {
                this.fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/vision/2025-test-field.json");
            } else {
                this.fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/vision/2025-reefscape.json");
            }
            
        } catch (Exception e) {System.err.println("April tag layout file not found");}

        if (Robot.isSimulation()) {
            visionSim.addAprilTags(this.fieldLayout);
            cameraSimProperties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            cameraSimProperties.setCalibError(0.25, 0.08);
            cameraSimProperties.setFPS(20);
            cameraSimProperties.setAvgLatencyMs(35);
            cameraSimProperties.setLatencyStdDevMs(5);

            cameraSim = new PhotonCameraSim(camera, cameraSimProperties);
            Translation3d robotToCameraTranslation = Constants.ROBOT_TO_CAMERA_POSE.getTranslation();
            Rotation3d robotToCameraRotation = Constants.ROBOT_TO_CAMERA_POSE.getRotation();
            Transform3d robotToCamera = new Transform3d(robotToCameraTranslation, robotToCameraRotation);
            visionSim.addCamera(cameraSim, robotToCamera);
        }
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

    public void updateSimulation() {
        visionSim.update(swerve.getPose());
        Field2d visionSimField = visionSim.getDebugField(); 
        SmartDashboard.putData("Vision Sim Field", visionSimField);
    }
}
