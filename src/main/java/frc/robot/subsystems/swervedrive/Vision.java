package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.awt.Desktop;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;


/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision
{
  
  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout                     = AprilTagFieldLayout.loadField(
      AprilTagFields.k2025Reefscape);
  /**
   * Photon Vision Simulation
   */
  public              VisionSystemSim     visionSim;
  /**
   * Count of times that the odom thinks we're more than 10meters away from the april tag.
   */
  private             double              longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private             Supplier<Pose2d>    currentPose;
  /**
   * Ambiguity defined as a value between (0,1). Used in {@link Vision#filterPose}.
   */
  private final double maximumAmbiguity = 0.25;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private             Field2d             field2d;


  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field)
  {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation())
    {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values())
      {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset)
  {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent())
    {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else
    {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive)
  {
    if (SwerveDriveTelemetry.isSimulation)
    {
      visionSim.update(swerveDrive.getPose());

    }
    for (Cameras camera : Cameras.values())
    {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent())
      {
        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                         pose.timestampSeconds,
                                         getEstimationStdDevs(camera));
      }
    }

  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   *  <li> No Pose Estimates could be generated</li>
   * <li> The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera)
  {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation())
    {
      Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
          est ->
              debugField
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }
  

  /**
   * The standard deviations of the estimated pose from {@link Vision#getEstimatedGlobalPose(Cameras)}, for use with
   * {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should only be used
   * when there are targets visible.
   *
   * @param camera Desired camera to get the standard deviation of the estimated pose.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Cameras camera)
  {
    var    poseEst    = getEstimatedGlobalPose(camera);
    var    estStdDevs = camera.singleTagStdDevs;
    var    targets    = getLatestResult(camera).getTargets();
    int    numTags    = 0;
    double avgDist    = 0;
    for (var tgt : targets)
    {
      var tagPose = camera.poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
      {
        continue;
      }
      numTags++;
      if (poseEst.isPresent())
      {
        avgDist += PhotonUtils.getDistanceToPose(poseEst.get().estimatedPose.toPose2d(), tagPose.get().toPose2d());
      }
    }
    if (numTags == 0)
    {
      return estStdDevs;
    }
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
    {
      estStdDevs = camera.multiTagStdDevs;
    }
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
    {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else
    {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    return estStdDevs;
  }

  /**
   * Filter pose via the ambiguity and find best estimate between all of the camera's throwing out distances more than
   * 10m for a short amount of time.
   *
   * @param pose Estimated robot pose.
   * @return Could be empty if there isn't a good reading.
   */
  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose)
  {
    if (pose.isPresent())
    {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (PhotonTrackedTarget target : pose.get().targetsUsed)
      {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity)
        {
          bestTargetAmbiguity = ambiguity;
        }
      }
      //ambiguity to high dont use estimate
      if (bestTargetAmbiguity > maximumAmbiguity)
      {
        return Optional.empty();
      }

      //est pose is very far from recorded robot pose
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1)
      {
        longDistangePoseEstimationCount++;

        //if it calculates that were 10 meter away for more than 10 times in a row its probably right
        if (longDistangePoseEstimationCount < 10)
        {
          return Optional.empty();
        }
      } else
      {
        longDistangePoseEstimationCount = 0;
      }
      return pose;
    }
    return Optional.empty();
  }

  /**
   * Get the latest result from a given Camera.
   *
   * @param camera Given camera to take the result from.
   * @return Photon result from sim or a real camera.
   */
  public PhotonPipelineResult getLatestResult(Cameras camera)
  {

    return Robot.isReal() ? camera.camera.getLatestResult() : camera.cameraSim.getCamera().getLatestResult();
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id)
  {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera)
  {
    PhotonTrackedTarget  target = null;
    PhotonPipelineResult result = getLatestResult(camera);
    if (result.hasTargets())
    {
      for (PhotonTrackedTarget i : result.getTargets())
      {
        if (i.getFiducialId() == id)
        {
          target = i;
        }
      }
    }
    return target;
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim()
  {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running photon vision on localhost.
   */
  private void openSimCameraViews()
  {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE))
    {
//      try
//      {
//        Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
//        Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
//        Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
//      } catch (IOException | URISyntaxException e)
//      {
//        e.printStackTrace();
//      }
    }
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField()
  {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values())
    {
      if (getLatestResult(c).hasTargets())
      {
        targets.addAll(getLatestResult(c).targets);
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets)
    {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent())
      {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Camera Enum to select each camera
   */
  enum Cameras
  {
    /**
     * Left Camera
     */
    LEFT_CAM("left",
             new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
             new Translation3d(Units.inchesToMeters(12.056),
                               Units.inchesToMeters(10.981),
                               Units.inchesToMeters(8.44)),
             VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    /**
     * Right Camera
     */
    RIGHT_CAM("right",
              new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
              new Translation3d(Units.inchesToMeters(12.056),
                                Units.inchesToMeters(-10.981),
                                Units.inchesToMeters(8.44)),
              VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    /**
     * Center Camera
     */
    CENTER_CAM("center",
               new Rotation3d(0, Units.degreesToRadians(18), 0),
               new Translation3d(Units.inchesToMeters(-4.628),
                                 Units.inchesToMeters(-10.687),
                                 Units.inchesToMeters(16.129)),
               VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

    /**
     * Latency alert to use when high latency is detected.
     */
    public final  Alert                        latencyAlert;
    /**
     * Camera instance for comms.
     */
    public final  PhotonCamera                 camera;
    /**
     * Pose estimator for camera.
     */
    public final  PhotonPoseEstimator          poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1>               singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1>               multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of the robot
     */
    private final Transform3d                  robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public        Matrix<N3, N1>               curStdDevs;
    /**
     * Estimated robot pose.
     */
    public        Optional<EstimatedRobotPose> estimatedRobotPose;
    /**
     * Simulated camera instance which only exists during simulations.
     */
    public        PhotonCameraSim              cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary queries.
     */
    public        List<PhotonPipelineResult>   resultsList       = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private       double                       lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix)
    {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout,
                                              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                              robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation())
      {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim)
    {
      if (Robot.isSimulation())
      {
        systemSim.addCamera(cameraSim, robotToCamTransform);
//        cameraSim.enableDrawWireframe(true);
      }
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose()
    {
      updateUnreadResults();
      return estimatedRobotPose;
    }
    private void updateUnreadResults()
    {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp    = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime        = Milliseconds.of(15).in(Seconds);
      for (PhotonPipelineResult result : resultsList)
      {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }
      if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
          (currentTimestamp - lastReadTimestamp) >= debounceTime)
      {
        resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        lastReadTimestamp = currentTimestamp;
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });
        if (!resultsList.isEmpty())
        {
          updateEstimatedGlobalPose();
        }
      }
    }
    private void updateEstimatedGlobalPose()
    {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList)
      {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets)
    {
      if (estimatedPose.isEmpty())
      {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else
      {
        // Pose present. Start running Heuristic
        var    estStdDevs = singleTagStdDevs;
        int    numTags    = 0;
        double avgDist    = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets)
        {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty())
          {
            continue;
          }
          numTags++;
          avgDist +=
              tagPose
                  .get()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0)
        {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else
        {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1)
          {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
          {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else
          {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }

}
