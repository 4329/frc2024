package frc.robot;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CommandGroups;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IndexFireCommand;
import frc.robot.commands.LightCommand;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.commands.armCommands.ArmDownCommand;
import frc.robot.commands.armCommands.ArmUpCommand;
import frc.robot.commands.armCommands.AutoZero;
import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.commands.driveCommands.ChangeFieldOrientCommand;
import frc.robot.commands.driveCommands.CoastCommand;
import frc.robot.commands.driveCommands.DriveByController;
import frc.robot.commands.driveCommands.DriveToTargetCommand;
import frc.robot.commands.driveCommands.ResetOdometryCommand;
import frc.robot.commands.elevatorCommands.ElevatorDownCommand;
import frc.robot.commands.elevatorCommands.ElevatorManualCommand;
import frc.robot.commands.elevatorCommands.ElevatorUpCommand;
import frc.robot.commands.shootCommands.ShootCommand;
import frc.robot.commands.shootCommands.ShooterAimCommand;
import frc.robot.commands.shootCommands.ShuffleBoardShootCommand;
import frc.robot.commands.visionCommands.LimDriveSetCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSusbsystem;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.HoorayConfig;

/* (including subsystems, commands, and button mappings) should be declared here
*/
public class RobotContainer {

  // The robot's subsystems
  private final Drivetrain m_robotDrive;

  final SendableChooser<Command> m_chooser;

  // The driver's controllers
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private final DriveByController m_drive;

  // Subsystem Declarations
  // private final VisionSubsystem visionSubsystem;
  // private final VisionSubsystem photonVisionSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final ShootSubsystem shootSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final IndexSubsystem indexSubsystem;
  private final ArmAngleSubsystem armAngleSubsystem;
  private final PoseEstimationSubsystem poseEstimationSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final LightsSusbsystem lightsSusbsystem;
  private final LineBreakSensorSubsystem lineBreakSensorSubsystem;
  private final LoggingSubsystem loggingSubsystem;

  // Command Declarations
  private final ExampleCommand exampleCommand;
  private final ResetOdometryCommand resetOdometryCommandForward;
  private final ResetOdometryCommand resetOdometryCommandBackward;
  private final ChangeFieldOrientCommand changeFieldOrientCommand;
  private final ShuffleBoardShootCommand shuffleBoardShootCommand;
  private final AutoZero autoZero;
  private final ElevatorManualCommand elevatorManualCommand;

  private final LightCommand lightCommandTwinkles;
  private final LightCommand lightCommandBlack;

  private final CenterOnTargetCommand centerOnTargetCommand;
  private final ShootCommand shootCommand;
  private final ShotReverseCommand shotReverseCommand;
  private final DriveToTargetCommand driveToTargetCommand;
  
  private final LimDriveSetCommand limDriveSetCommand;
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @param drivetrain
   */

  public RobotContainer(Drivetrain drivetrain) {
    m_robotDrive = drivetrain;

    operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
    driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    m_drive = new DriveByController(m_robotDrive, driverController);

    // Subsystem Instantiations
    if (HoorayConfig.gimmeConfig().getUsesPhotonVision()) {
      visionSubsystem = new PhotonVisionSubsystem();
      Shuffleboard.getTab("Config").add("Camera", "Photon");
    } else {
      visionSubsystem = new LimlihSubsystem();
      Shuffleboard.getTab("Config").add("Camera", "Limlih");
    }
    shootSubsystem = new ShootSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    indexSubsystem = new IndexSubsystem();
    armAngleSubsystem = new ArmAngleSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    lightsSusbsystem = new LightsSusbsystem();
    lineBreakSensorSubsystem = new LineBreakSensorSubsystem();
    poseEstimationSubsystem = new PoseEstimationSubsystem(drivetrain, visionSubsystem, armAngleSubsystem);
    loggingSubsystem = new LoggingSubsystem(armAngleSubsystem, elevatorSubsystem, indexSubsystem, intakeSubsystem, lineBreakSensorSubsystem, poseEstimationSubsystem, shootSubsystem);
    
    // commands for auto
    NamedCommands.registerCommand("intake", CommandGroups.intakeFull(intakeSubsystem, indexSubsystem));
    NamedCommands.registerCommand("stop", new InstantCommand(() -> drivetrain.stop()));

    // Command Instantiations
    exampleCommand = new ExampleCommand();
    resetOdometryCommandForward = new ResetOdometryCommand(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)),
        drivetrain);
    resetOdometryCommandBackward = new ResetOdometryCommand(new Pose2d(new Translation2d(), new Rotation2d(0.0)),
        drivetrain);
    changeFieldOrientCommand = new ChangeFieldOrientCommand(m_drive);
    centerOnTargetCommand = new CenterOnTargetCommand(visionSubsystem, m_robotDrive, 7, driverController);
    shootCommand = new ShootCommand(shootSubsystem);
    shuffleBoardShootCommand = new ShuffleBoardShootCommand(shootSubsystem);
    autoZero = new AutoZero(elevatorSubsystem, armAngleSubsystem);
    shotReverseCommand = new ShotReverseCommand(shootSubsystem);

    elevatorManualCommand = new ElevatorManualCommand(elevatorSubsystem, () -> driverController.getLeftTriggerAxis(), () -> driverController.getRightTriggerAxis());
    lightCommandTwinkles = new LightCommand(lightsSusbsystem, 0.51);
    lightCommandBlack = new LightCommand(lightsSusbsystem, 0.99);
    limDriveSetCommand = new LimDriveSetCommand(visionSubsystem, drivetrain, poseEstimationSubsystem);
    // shootSubsystem.setDefaultCommand(shuffleBoardShootCommand);
    driveToTargetCommand = new DriveToTargetCommand(drivetrain, visionSubsystem, 4, -3);
    armAngleSubsystem.setDefaultCommand(new ShooterAimCommand(visionSubsystem,
    armAngleSubsystem));

    // armAngleSubsystem.setDefaultCommand(new ShooterAimCommand(limlihSubsystem,
    // armAngleSubsystem));

    m_chooser = new SendableChooser<>();
    initializeCamera();
    configureButtonBindings();
    configureAutoChooser(drivetrain);

  }

  /**
   * Creates and establishes camera streams for the shuffleboard ~Ben
   */
  HttpCamera limelight;

  private void initializeCamera() {

    // CameraServer.startAutomaticCapture();
    // // System.out.println(CameraServer.getVideo());
    // VideoSource[] enumerateSources = VideoSource.enumerateSources();
    // System.out.println(enumerateSources[0].getName());
    // if (enumerateSources.length > 0 &&
    // enumerateSources[0].getName().contains("USB")) {
    // Shuffleboard.getTab("RobotData").add("Camera",
    // enumerateSources[0]).withPosition(5, 0).withSize(3, 3)
    // .withWidget(BuiltInWidgets.kCameraStream);
    // }

    limelight = new HttpCamera("Limelight", HoorayConfig.gimmeConfig().getLimelighturl());
    System.out.println(HoorayConfig.gimmeConfig().getLimelighturl());
    CameraServer.startAutomaticCapture(limelight);
    // Shuffleboard.getTab("RobotData").add("Limelight Camera",
    // limelight).withPosition(2, 0).withSize(2, 2)
    // .withWidget(BuiltInWidgets.kCameraStream);
  }

  /* Autonomous :D */
  private Map<String, Command> createEventMap() {
    Map<String, Command> eventMap = new HashMap<>();
    eventMap.put("Example Command", new ExampleCommand());
    // eventMap.put("shootyshootshoot", CommandGroups.aimAndShoot(shootSubsystem,
    // m_robotDrive, indexSubsystem, limlihSubsystem, driverController,
    // armAngleSubsystem).withTimeout(10));
    // eventMap.put("knomknom", CommandGroups.intakeFull(intakeSubsystem,
    // indexSubsystem).withTimeout(5));

    return eventMap;
  }

  private void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
        poseEstimationSubsystem::getPathPlannerStuff,
        m_robotDrive::resetOdometry,
        m_robotDrive::getChassisSpeed,
        m_robotDrive::setModuleStates,
        new HolonomicPathFollowerConfig(
            new PIDConstants(Constants.AutoConstants.kPXController),
            new PIDConstants(Constants.AutoConstants.kPThetaController),
            Constants.AutoConstants.kMaxSpeed,
            0.4,
            new ReplanningConfig()),
        () -> {
          return false;
        },
        m_robotDrive);
  }

  // private SwerveAutoBuilder createAutoBuilder() {
  //
  // SwerveAutoBuilder autoBuilder = new GimmeSwerve(
  //
  // m_robotDrive::getPose, // Pose2d supplier
  // m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at
  // the beginning of auto
  // Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
  // new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID
  // constants to correct for translation
  // // error (used to create the X and Y PID
  // // controllers)
  // new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // PID
  // constants to correct for rotation
  // // error (used to create the rotation
  //
  // m_robotDrive::setModuleStates, // Module states consumer used to output to
  // the drive subsystem
  // createEventMap(),
  // true, // Should the path be automatically mirrored depending on alliance
  // color.
  // // Optional, defaults to true
  // m_robotDrive // The drive subsystem. Used to properly set the requirements of
  // path following
  // // commands
  // );
  //
  // return autoBuilder;
  // }

  /**
   * 
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
   * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Controller
    driverController.rightTrigger().whileTrue(elevatorManualCommand);
    driverController.leftTrigger().whileTrue(elevatorManualCommand);

    driverController.rightBumper().whileTrue(CommandGroups.holdShot(shootSubsystem, m_robotDrive, visionSubsystem, driverController, armAngleSubsystem)).toggleOnFalse(new IndexFireCommand(indexSubsystem, shootSubsystem));
    driverController.leftBumper().whileTrue(shotReverseCommand);
    driverController.start().whileTrue(CommandGroups.intakeWithLineBreakSensor(intakeSubsystem, indexSubsystem, lineBreakSensorSubsystem));
    driverController.back().onTrue(changeFieldOrientCommand);

    driverController.a().whileTrue(exampleCommand);
    driverController.b().whileTrue(exampleCommand);
    driverController.x().whileTrue(new ArmUpCommand(armAngleSubsystem));    
    driverController.y().whileTrue(new ArmDownCommand(armAngleSubsystem));

    // driverController.povUp().whileTrue(CommandGroups.aimAndShoot(shootSubsystem, m_robotDrive, indexSubsystem, visionSubsystem, driverController, armAngleSubsystem)).toggleOnFalse(new ArmAngleCommand(armAngleSubsystem, ArmAngle.ZERO));
    driverController.povRight().whileTrue(CommandGroups.intakeFull(intakeSubsystem, indexSubsystem));
    driverController.povLeft().whileTrue(CommandGroups.outakeFull(intakeSubsystem, indexSubsystem));
    // driverController.povDown().whileTrue(exampleCommand);

    driverController.rightStick().whileTrue(exampleCommand);
    driverController.leftStick().whileTrue(resetOdometryCommandForward); // field orient

    // Operator Controller
    operatorController.rightTrigger().whileTrue(exampleCommand);
    operatorController.leftTrigger().whileTrue(exampleCommand);
  
    operatorController.rightBumper().whileTrue(new ArmUpCommand(armAngleSubsystem));
          operatorController.leftBumper().whileTrue(new ArmDownCommand(armAngleSubsystem)); 


    operatorController.start().whileTrue(exampleCommand); 
    operatorController.back().onTrue(changeFieldOrientCommand);

    operatorController.a().whileTrue(CommandGroups.intakeFull(intakeSubsystem, indexSubsystem));
    operatorController.b().whileTrue(CommandGroups.outakeFull(intakeSubsystem, indexSubsystem));
    operatorController.x().whileTrue(new ElevatorDownCommand(elevatorSubsystem));
    operatorController.y().whileTrue(new ElevatorUpCommand(elevatorSubsystem));//hi jonny was here

    operatorController.povUp().onTrue(new ArmAngleCommand(armAngleSubsystem, ArmAngle.ZERO));
    operatorController.povRight().onTrue(exampleCommand);
    operatorController.povLeft().onTrue(exampleCommand);
    operatorController.povDown().onTrue(new ArmAngleCommand(armAngleSubsystem, ArmAngle.HORIZONTAL));
  }

  // jonathan was here today 2/3/2023
  /* Pulls autos and configures the chooser */
  // SwerveAutoBuilder swerveAutoBuilder;

  private void configureAutoChooser(Drivetrain drivetrain) {
    configureAutoBuilder();

    // swerveAutoBuilder = createAutoBuilder();
    File pathPlannerDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner");
    pathPlannerDirectory = new File(pathPlannerDirectory, "autos");

    for (File pathFile : pathPlannerDirectory.listFiles()) {

      // System.out.println(pathFile);

      if (pathFile.isFile() && pathFile.getName().endsWith(".auto")) {

        String name = pathFile.getName().replace(".auto", "");

        // List<PathConstraints> constraints = getPathConstraints(name);

        // List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(name,
        // constraints);

        Command pathCommand = new PathPlannerAuto(name);
        if (name.endsWith("BalanceAuto")) {
          // m_chooser.addOption(name, new SequentialCommandGroup(pathCommand, new
          // BalanceCommand(m_robotDrive, balanceSubsystem).withTimeout(12)));
        } else {
          m_chooser.addOption(name, pathCommand);
        }
      }

      SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(shootSubsystem::setVoltage, shootSubsystem::getData, shootSubsystem));
      m_chooser.addOption("yes", new SequentialCommandGroup(
        sysIdRoutine.dynamic(Direction.kForward),
        new WaitCommand(5),
        sysIdRoutine.dynamic(Direction.kReverse),
        new WaitCommand(5),
        sysIdRoutine.quasistatic(Direction.kForward),
        new WaitCommand(5),
        sysIdRoutine.quasistatic(Direction.kReverse)
      ));
    }
    // m_chooser.addOption("Example Path", new PathPlannerAuto("New Auto"));

    Shuffleboard.getTab("RobotData").add("SelectAuto", m_chooser).withSize(3, 2).withPosition(0, 0);
  }

  public void robotInit() {

    limDriveSetCommand.schedule();

  }
  private void getData(SysIdRoutineLog sysIdRoutineLog) {
    sysIdRoutineLog.motor("Shoot");
  }

  public void autonomousInit() {

    limDriveSetCommand.schedule();

  }

  public void teleopInit() {
    m_robotDrive.setDefaultCommand(m_drive);
    // autoZero.schedule();
  }

  public void autonomousPeriodic() {

    if (!limDriveSetCommand.isScheduled())
      limDriveSetCommand.schedule();

  }

  public void teleopPeriodic() {

    if (!limDriveSetCommand.isScheduled())
      limDriveSetCommand.schedule();

  }

  /**
   * @return Selected Auto
   */
  public Command getAuto() {

    return m_chooser.getSelected();
  }

  public void configureTestMode() {

    m_robotDrive.setDefaultCommand(new CoastCommand(m_robotDrive));
  }

}