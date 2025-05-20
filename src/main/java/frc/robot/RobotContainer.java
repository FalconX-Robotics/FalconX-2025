// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.auto.AutoParser;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.auto.GoToArmPosition;
import frc.robot.commands.auto.GoToArmPosition.Position;
import frc.robot.commands.climb.ClimbCommand;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.intake.GrabCoral;
import frc.robot.commands.intake.Release;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.ChangeSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Util;

import java.time.LocalDateTime;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  public static RobotContainer INSTANCE;

  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  public final Settings settings = new Settings(driverXbox, operatorXbox);
  public final Intake intake = new Intake();
  public final Arm arm = new Arm();
  public final Elevator elevator = new Elevator();
  
  public final Climber climber = new Climber();

  public final Command autoCommand;

  UsbCamera intakeCam;
  
  AbsoluteFieldDrive absFieldDrive = new AbsoluteFieldDrive(swerve, 
  () -> MathUtil.applyDeadband(settings.driverSettings.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
  () -> MathUtil.applyDeadband(-settings.driverSettings.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
  () -> Math.atan2(-settings.driverSettings.getRightX(), settings.driverSettings.getRightY()));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command zeroMotion = swerve.driveCommand(
    ()-> 0.0, ()->0.0, ()->0.0);
  
  Command driveFieldOrientedDirectAngle = swerve.driveCommand(
      () -> MathUtil.applyDeadband(-settings.driverSettings.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-settings.driverSettings.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -settings.driverSettings.getRightX(),
      () -> -settings.driverSettings.getRightY());

  Command driveInputs = new ParallelCommandGroup(new ChangeSpeed(swerve), swerve.driveInputs(()->-settings.driverSettings.getLeftY(), ()->-settings.driverSettings.getLeftX(), ()->-settings.driverSettings.getRightX()));

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command absoluteDrive = new AbsoluteDrive(swerve, 
    () -> MathUtil.applyDeadband(settings.driverSettings.getLeftY() * -1, OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(settings.driverSettings.getLeftX() * -1, OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(settings.driverSettings.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    () -> MathUtil.applyDeadband(settings.driverSettings.getRightY(), OperatorConstants.RIGHT_X_DEADBAND));

  Command driveFieldOrientedAnglularVelocity = swerve.driveCommand(
      () -> MathUtil.applyDeadband(settings.driverSettings.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(settings.driverSettings.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> settings.driverSettings.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = swerve.simDriveCommand(
      () -> MathUtil.applyDeadband(settings.driverSettings.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(settings.driverSettings.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRawAxis(2));


  public RobotContainer()
  {
    RobotContainer.INSTANCE = this;
    Util.setStartTime(LocalDateTime.now());
    DataLogManager.start(Filesystem.getOperatingDirectory() + "/logs", Util.getLogFilename());
    // NamedCommands.registerCommand("Outtake", new Release(intake, settings));
    // NamedCommands.registerCommand("Arm L2", new GoToArmPosition(Position.L2, arm, elevator));    NamedCommands.registerCommand("Go To L3", new GoToArmPosition(Position.L3, arm, elevator));
    NamedCommands.registerCommand("Go to L2", new GoToArmPosition(Position.L2, arm, elevator));
    NamedCommands.registerCommand("Outtake", new Release(intake, settings));
    NamedCommands.registerCommand("Outtake Slow", new Release(intake, 0.35));
    NamedCommands.registerCommand("Intake Position", new GoToArmPosition(Position.INTAKE, arm, elevator));
    NamedCommands.registerCommand("Travel", new GoToArmPosition(Position.TRAVEL, arm, elevator));
    NamedCommands.registerCommand("Intake", new GrabCoral(intake, settings));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    swerve.setupPathPlanner();
    // Configure thse trigger bindings
    autoCommand = AutoParser.parseAuto(Filesystem.getDeployDirectory() + "/auto.mfd");
    configureBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    if (!Robot.isSimulation()) {
      intakeCam = CameraServer.startAutomaticCapture();
      intakeCam.setFPS(20);
    }
  }

  public void robotPeriodic() {
    SmartDashboard.putBoolean("inverte", settings.driverSettings.inverted);
    // Mat image = new Mat();
    // cvSink.grabFrame(image);
    // if (!image.empty()) {
    //   output.putFrame(image);
    //   System.out.println("camera periodic");
    // }
    // Mat transposed = new Mat();
    // Core.transpose(image, transposed);
    // Mat rotated = new Mat();
    // Core.flip(transposed, rotated, 1);
    
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
    // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    settings.driverSettings.speedModeButton.whileTrue(driveInputs);
    settings.driverSettings.invertButton.onTrue(Commands.runOnce(()-> {settings.driverSettings.inverted = !settings.driverSettings.inverted;}, climber));
    // settings.driverSettings.resetAuto.onTrue(this.autoCommand);
    swerve.setDefaultCommand(driveFieldOrientedDirectAngle);

    // settings.armSettings.overrideArm.whileTrue(new ChangeIntakeAngle(arm, operatorXbox));
    settings.operatorSettings.coralIntakeButton.whileTrue(new GrabCoral(intake, settings));
    settings.operatorSettings.releaseButton.whileTrue(new Release(intake, settings));
    arm.setDefaultCommand(new MoveArm(arm, operatorXbox));
    
    settings.operatorSettings.climbButton.whileTrue(new ClimbCommand(climber, swerve, false, settings));
    settings.operatorSettings.unClimbButton.whileTrue(new ClimbCommand(climber, swerve, true, settings));
    settings.operatorSettings.moveToL2.whileTrue(new GoToArmPosition(Position.L2, arm, elevator));
    settings.operatorSettings.moveToL3.whileTrue(new GoToArmPosition(Position.L3, arm, elevator));

    settings.operatorSettings.travelButton.whileTrue(new GoToArmPosition(Position.TRAVEL, arm, elevator));
    settings.operatorSettings.moveToIntake.whileTrue(new GoToArmPosition(Position.INTAKE, arm, elevator));
    settings.operatorSettings.moveToL2.whileTrue(new GoToArmPosition(Position.L2, arm, elevator));
    settings.operatorSettings.moveToL3.whileTrue(new GoToArmPosition(Position.L3, arm, elevator));
    settings.operatorSettings.moveToLoAlgae.whileTrue(new GoToArmPosition(Position.LO_ALGAE, arm, elevator));
    settings.operatorSettings.moveToHiAlgae.whileTrue(new GoToArmPosition(Position.HI_ALGAE, arm, elevator));

    elevator.setDefaultCommand(new ManualElevator(elevator, operatorXbox));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return this.autoCommand;
    // returfn swerve.driveAndSpin();
    // return autoChooser.getSelected();
    // return drivebase.driveToDistanceCommand(1, 0.1);
    // return drivebase.driveToDistanceCommand(200, 0.5);
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("PID Test");
  }

  public void setDriveMode()
  {
    elevator.setSetpoint(0);
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {

    // swerve.setMotorBrake(brake);
  }
}
