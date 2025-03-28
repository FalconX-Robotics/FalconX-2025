// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.commands.auto.GoToArmPosition;
import frc.robot.commands.auto.GoToArmPosition.Position;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{

  /**
   * PhotonVision class to keep an accurate odometry.
   */
  private       Vision  vision;
  /**
   * Swerve drive object.
   */
  private final SwerveDrive         swerveDrive;

  private DoubleLogEntry xPositionLog = new DoubleLogEntry(DataLogManager.getLog(), "xPosition");
  private BooleanLogEntry teleopLog = new BooleanLogEntry(DataLogManager.getLog(), "Teleop");
  private BooleanLogEntry autoLog = new BooleanLogEntry(DataLogManager.getLog(), "Autonomous");
  private DoubleLogEntry rotationLog = new DoubleLogEntry(DataLogManager.getLog(), "Rotation");
  /**
   * AprilTag field layout.
   */
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
  /**
   * Enable vision odometry updates while driving.
   */
  private final boolean visionDriveTest = true;
  public boolean speedMode = false;
  public boolean allowVisionPose = true;
  public boolean climbing = false;

  public SwerveDrive getSwerveDrive() {return swerveDrive;}

  // private DataLog dataLog = DataLogManager.getLog();
  // private DoubleLogEntry xVelocityLog;
  // private DoubleLogEntry yVelocityLog;
  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
    System.out.println("\"conversionFactors\": {");
    System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
    System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
    System.out.println("}");
    

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, Constants.STARTING_POSE);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders  when they are not moving.
    swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    if (visionDriveTest)
    {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }
    // setupPathPlanner();
    // setupPathPlanner();
  }
  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.MAX_SPEED, Constants.STARTING_POSE);
  }

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision()
  {
    PhotonCamera camera = new PhotonCamera("Limelight");
    vision = new Vision(camera, this);
  }

  public Vision getVision() {
    return vision;
  }

  @Override
  public void periodic() {
    xPositionLog.append(getPose().getX());
    teleopLog.append(DriverStation.isTeleopEnabled());
    autoLog.append(DriverStation.isAutonomousEnabled());
    rotationLog.append(getYaw().getDegrees());
    Optional<Pose2d> cameraPose = vision.getFieldPose();
    if (cameraPose.isPresent()) {
      // swerveDrive.resetOdometry(cameraPose.get());
    }
    
    // vision.updatePoseEstimation(swerveDrive);
    SmartDashboard.putNumber("Y velocity", getRobotVelocity().vyMetersPerSecond);
    SmartDashboard.putNumber("X velocity", getRobotVelocity().vxMetersPerSecond);
    SmartDashboard.putNumber("Angle velocity", getRobotVelocity().omegaRadiansPerSecond);
    SmartDashboard.putNumber("Gyro Value",Math.toDegrees(swerveDrive.getGyro().getRawRotation3d().getZ()));
    // System.out.println("Robot Speed " + getRobotVelocity());
    // System.out.println("Max Velocity " + swerveDrive.getMaximumChassisVelocity());
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (true)
    {
      Optional<Transform3d> tagPose = vision.getTagPose(4);
      if (tagPose.isPresent()) {
        double targetYaw = tagPose.get().getRotation().getZ();
        // SmartDashboard.putNumber("robot angle", Math.toDegrees(getYaw().getRadians()));
        SmartDashboard.putNumber("target angle", Math.toDegrees(targetYaw));
      }
      

      if (cameraPose.isPresent()) {
        Pose2d estimatedPose = cameraPose.get();
        SmartDashboard.putNumber("Estimated Robot Pose X", estimatedPose.getX());
        SmartDashboard.putNumber("Estimated Robot Pose Y", estimatedPose.getY());
        SmartDashboard.putNumber("Estimated Robot Angle", estimatedPose.getRotation().getDegrees());
        // SmartDashboard.putNumber("Estimated Robot Pose Z", estimatedPose.getZ());
      }
      // Optional<EstimatedRobotPose> poseEst = vision.getEstimatedGlobalPose(Cameras.LIMELIGHT);
      // if (poseEst.isPresent()) {
      //   swerveDrive.addVisionMeasurement(poseEst.get().estimatedPose.toPose2d(), poseEst.get().timestampSeconds);
      // }
      if (!climbing) {
        swerveDrive.updateOdometry();
      }
      // vision.updatePoseEstimation(swerveDrive);
    }
    SmartDashboard.putNumber("Robot Field X", getPose().getX());
    SmartDashboard.putNumber("Robot Field Y", getPose().getY());
    SmartDashboard.putNumber("Robot Field Angle", getPose().getRotation().getDegrees());
    ChassisSpeeds speed = getRobotVelocity();

    SmartDashboard.putNumber("X Velocity", speed.vxMetersPerSecond);
    SmartDashboard.putNumber("Y Velocity", speed.vyMetersPerSecond);
    SmartDashboard.putNumber("Rotation Velocity", speed.omegaRadiansPerSecond);
    //not moving
    if (MathUtil.isNear(0, speed.vxMetersPerSecond, 0.0001) && MathUtil.isNear(0, speed.vyMetersPerSecond, 0.0001) && MathUtil.isNear(0, speed.omegaRadiansPerSecond, 0.0001)) {
      Optional<Pose2d> visionPose = vision.getFieldPose();
      if (visionPose.isPresent()) {
        // System.out.println("update odometry");

        resetOdometry(visionPose.get());
      }
    }
  }

  @Override
  public void simulationPeriodic()
  {
    vision.updateSimulation();
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      final boolean enableFeedforward = true;
      
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                                );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            // var alliance = DriverStation.getAlliance();
            // if (alliance.isPresent())
            // {
            //   return alliance.get() == DriverStation.Alliance.Red;
            // }
            // return false;
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                            );
          Pathfinding.setPathfinder(new LocalADStar());

    } catch (Exception e) {

    }
  }

  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @param camera {@link PhotonCamera} to communicate with.
   * @return A {@link Command} which will run the alignment.
   */
  public Command aimAtTarget(PhotonCamera camera)
  {

    return run(() -> {
      PhotonPipelineResult result = camera.getLatestResult();
      if (result.hasTargets())
      {
        drive(getTargetSpeeds(0,
                              0,
                              Rotation2d.fromDegrees(result.getBestTarget()
                                                           .getYaw()))); // Not sure if this will work, more math may be required.
      }
    });
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  public Command driveInputs(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      double x = translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity();
      double y = translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity();
      double angle = angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity();
      if (speedMode) {
        x *= Constants.DrivebaseConstants.SPEED_MODE_SCALE;
        y *= Constants.DrivebaseConstants.SPEED_MODE_SCALE;
        angle *= Constants.DrivebaseConstants.SPEED_MODE_SCALE;
      }
      swerveDrive.drive(new Translation2d(x, y),
                        angle,
                        true,
                        true);
    });
  }

  public Command driveAndSpin() {
    return run(()-> {
      driveFieldOriented(new ChassisSpeeds(0.5, 0, Math.PI));
    });
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
// Create the constraints to use while  
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0)
                                    ).andThen(()-> {
                                      Elastic.Notification notification = new Elastic.Notification(NotificationLevel.INFO, "", "Drove to " + pose);
                                      Elastic.sendNotification(notification);
                                    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      double xSpeed = translationX.getAsDouble();
      double ySpeed = translationY.getAsDouble();
      if (speedMode) {
        xSpeed *= Constants.DrivebaseConstants.SPEED_MODE_SCALE;
        ySpeed *= Constants.DrivebaseConstants.SPEED_MODE_SCALE;
      }
      // System.out.("position" + getPose().getTranslation());
      // System.out.priprintlnntln("velocity" + getFieldVelocity());

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(xSpeed,
                                                                                 ySpeed), 0.8);
      // Make the robot move
      SmartDashboard.putNumber("Controller Angle", scaledInputs.getAngle().getDegrees());
      SmartDashboard.putNumber("Controller X Speed", scaledInputs.getX());
      SmartDashboard.putNumber("Controller Y Speed", scaledInputs.getY());
      
      ChassisSpeeds speeds = swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
        headingX.getAsDouble(),
        headingY.getAsDouble(),
        swerveDrive.getOdometryHeading().getRadians(),
        swerveDrive.getMaximumChassisVelocity());
      if (MathUtil.applyDeadband(headingX.getAsDouble(), 0.1) == 0 && MathUtil.applyDeadband(headingY.getAsDouble(), 0.1) == 0) speeds.omegaRadiansPerSecond = 0;
      // speeds.omegaRadiansPerSecond *= 3;
      // speeds.omegaRadiansPerSecond = headingX.getAsDouble();
      SmartDashboard.putNumber("Robot Movement Speeds X", speeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Robot Movement Speeds Y", speeds.vyMetersPerSecond);
      SmartDashboard.putNumber("Robot Movement Speeds Angle", speeds.omegaRadiansPerSecond);
      driveFieldOriented(speeds);
    });
  }

  // public Command spinAndMove() {
  //   AutoBuilder.
  // }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation     Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
  {
    
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      
      double xSpeed = translationX.getAsDouble();
      double ySpeed = translationY.getAsDouble();
      if (speedMode) {
        xSpeed *= Constants.DrivebaseConstants.SPEED_MODE_SCALE;
        ySpeed *= Constants.DrivebaseConstants.SPEED_MODE_SCALE;
      }
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xSpeed,
                                                                      ySpeed,
                                                                      rotation.getAsDouble() * Math.PI,
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return Commands.none();
    // return SwerveDriveTest.generateSysIdCommand(
    //     SwerveDriveTest.setDriveSysIdRoutine(
    //         new Config(),
    //         this, swerveDrive, 12),
    //     3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    System.out.println("centered");
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return Commands.deferredProxy(
        () -> Commands.run(() -> {
          drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0));
          System.out.println(swerveDrive.getPose().getTranslation());
        }, this)
                      .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                                   distanceInMeters)
                                 );
}

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param  kS  the static gain of the feedforward
   * @param  kV  the velocity gain of the feedforward
   * @param  kA  the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  { 
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      double xSpeed = translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity();
      double ySpeed = translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity();
      if (speedMode) {
        xSpeed *= Constants.DrivebaseConstants.SPEED_MODE_SCALE;
        ySpeed *= Constants.DrivebaseConstants.SPEED_MODE_SCALE;
      }
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            xSpeed,
                            ySpeed), 0.8),
                            Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
      //System.out.println(getRobotVelocity().omegaRadiansPerSecond);
                        //Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity()
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    // for (SwerveModule sm : swerveDrive.getModules()) {
    //   System.out.println(sm.getState().angle);
    // }
    
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    Pose2d pose = getPose();
    resetOdometry(new Pose2d(pose.getX(), pose.getY(), new Rotation2d()));
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    return getTargetSpeeds(xInput, yInput, headingX, headingY, Units.feetToMeters(1));
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY, double maxSpeed) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    ChassisSpeeds speeds = swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        // System.out.println(speeds);
                                                        Constants.MAX_SPEED);
    return speeds;
  }
  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }
  public Rotation2d getYaw()
  {
    return swerveDrive.getOdometryHeading();
    // return swerveDrive.getYaw();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }
}