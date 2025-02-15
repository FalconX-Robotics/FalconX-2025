// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.ChangeSpeed;
import frc.robot.commands.swervedrive.drivebase.LineUpReef;
import frc.robot.commands.swervedrive.drivebase.PointToTarget;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Util;
import swervelib.encoders.SwerveAbsoluteEncoder;

import java.io.File;
import java.nio.file.FileSystem;
import java.time.LocalDateTime;
import java.util.function.BooleanSupplier;
import java.util.logging.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(swerve,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.LEFT_X_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);


  AbsoluteFieldDrive absFieldDrive = new AbsoluteFieldDrive(swerve, 
  () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
  () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
  () -> Math.atan2(-driverXbox.getRightX(), driverXbox.getRightY()));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command zeroMotion = swerve.driveCommand(
    ()-> 0.0, ()->0.0, ()->0.0);
  
  Command driveFieldOrientedDirectAngle = swerve.driveCommand(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverXbox.getRightX(),
      () -> -driverXbox.getRightY());

  Command driveInputs = swerve.driveInputs(()->driverXbox.getLeftY(), ()->driverXbox.getLeftX(), ()->driverXbox.getRightX());

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command absoluteDrive = new AbsoluteDrive(swerve, 
    () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    () -> MathUtil.applyDeadband(driverXbox.getRightY(), OperatorConstants.RIGHT_X_DEADBAND));

  Command driveFieldOrientedAnglularVelocity = swerve.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = swerve.simDriveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRawAxis(2));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  
  public RobotContainer()
  { 
    Util.setStartTime(LocalDateTime.now());
    DataLogManager.start(Filesystem.getOperatingDirectory() + "/logs", Util.getLogFilename());
    swerve.setupPathPlanner();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    if (DriverStation.isTest())
    {
      // driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      // driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.leftBumper().onTrue(drivebase.driveToPose(new Pose2d(1,1,new Rotation2d(180))));
      // driverXbox.rightBumper().onTrue(Commands.none());
      // drivebase.setDefaultCommand(absoluteDrive);
    } else
    {
      Command driveToPointA = swerve.driveToPose(new Pose2d(3,2,new Rotation2d(Math.PI/2)));
      Command driveToPointB = swerve.driveToPose(new Pose2d(1,1,new Rotation2d(Math.PI)));
      Command driveToPointC = swerve.driveToPose(new Pose2d(2,2,new Rotation2d(0)));
      // driverXbox.a().onTrue(getAutonomousCommand());
      // driverXbox.x().whileTrue(swerve.aimAtTarget(swerve.getVision().camera));
      // // driverXbox.b().whileTrue(new ChangeSpeed(swerve));
      driverXbox.rightTrigger().whileTrue(new ChangeSpeed(swerve));
      // driverXbox.y().whileTrue(new PointToTarget(swerve));
      // driverXbox.back().onTrue(driveToPointA);
      // driverXbox.leftBumper().onTrue(driveToPointB);
      // driverXbox.rightBumper().onTrue(new LineUpReef(swerve, 3, LineUpReef.Side.LEFT));
      // // driverXbox.start().onTrue(driveToPointA.andThen(driveToPointB.andThen(driveToPointC)));
      // driverXbox.leftTrigger().onTrue(new LineUpReef(swerve, 4, LineUpReef.Side.RIGHT));
      swerve.setDefaultCommand(driveFieldOrientedDirectAngle);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // return swerve.driveAndSpin();
    return autoChooser.getSelected();
    // return drivebase.driveToDistanceCommand(1, 0.1);
    // return drivebase.driveToDistanceCommand(200, 0.5);
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("PID Test");
  }

  public void setDriveMode()
  {
    
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    swerve.setMotorBrake(brake);
  }
}
