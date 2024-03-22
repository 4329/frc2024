package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.commands.LightCommands.LightBlankCommand;
import frc.robot.commands.LightCommands.LightFastProgressCommand;
import frc.robot.commands.LightCommands.LightProgressCommand;
import frc.robot.commands.LightCommands.LightRambowCommand;
// import frc.robot.commands.LightCommands.LightIndividualCommand;
import frc.robot.commands.LightCommands.LightsOnCommand;
import frc.robot.commands.armCommands.ArmDownCommand;
import frc.robot.commands.armCommands.ArmUpCommand;
import frc.robot.commands.armCommands.AutoZero;
import frc.robot.commands.armCommands.MoveArmCommand;
import frc.robot.commands.armCommands.ShootAmpCommand;
import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.commands.driveCommands.ChangeFieldOrientCommand;
import frc.robot.commands.driveCommands.CoastCommand;
import frc.robot.commands.driveCommands.DriveByController;
import frc.robot.commands.driveCommands.DriveToTargetCommand;
import frc.robot.commands.driveCommands.PPCenterOnTarget;
import frc.robot.commands.driveCommands.ResetOdometryCommand;
import frc.robot.commands.elevatorCommands.ElevatorManualCommand;
import frc.robot.commands.elevatorCommands.ElevatorToAmpCommand;
import frc.robot.commands.indexCommands.IndexCommand;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.commands.indexCommands.IndexSensorCommand;
import frc.robot.commands.intakeOuttakeCommands.IntakeSensorCommand;
import frc.robot.commands.intakeOuttakeCommands.ToggleIntakeCommand;
import frc.robot.commands.shootCommands.AutoSideShotCommand;
import frc.robot.commands.shootCommands.ShootCommand;
import frc.robot.commands.shootCommands.ShotReverseCommand;
import frc.robot.commands.shootCommands.ShuffleBoardShootCommand;
import frc.robot.commands.visionCommands.CheckLimelightCommand;
import frc.robot.commands.visionCommands.LimDriveSetCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.CommandLoginator;
import frc.robot.utilities.HoorayConfig;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

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
  private final LineBreakSensorSubsystem lineBreakSensorSubsystem;
  private final LoggingSubsystem loggingSubsystem;
  private final LightSubsystem lightSubsystem;

  // Command Declarations
  private final ExampleCommand exampleCommand;
  private final ResetOdometryCommand resetOdometryCommandForward;
  private final ResetOdometryCommand resetOdometryCommandBackward;
  private final ChangeFieldOrientCommand changeFieldOrientCommand;
  private final AutoZero autoZero;
  private final ElevatorManualCommand elevatorManualCommand;
  private final ToggleIntakeCommand toggleIntakeCommand;

  private final LightProgressCommand lightProgressCommand;
  private final LightBlankCommand lightBlankCommand;
  private final LightsOnCommand lightsOnCommand;
  private final LightRambowCommand lightRambowCommand;
  private final LightFastProgressCommand lightFastProgressCommand;

  private final CenterOnTargetCommand centerOnTargetCommand;
  private final ShootCommand shootCommand;
  private final ShotReverseCommand shotReverseCommand;
  private final ShootAmpCommand shootAmpCommand;
  private final ElevatorToAmpCommand elevatorToAmpCommand;
  private final DriveToTargetCommand driveToTargetCommand;

  private final LimDriveSetCommand limDriveSetCommand;
  private final GenericEntry alliance;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @param drivetrain
   * @param lightSubsystem
   */
  public RobotContainer(
      Drivetrain drivetrain,
      CheckLimelightCommand checkLimelightCommand,
      LightSubsystem lightSubsystem) {
    m_robotDrive = drivetrain;

    operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
    driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    m_drive = new DriveByController(m_robotDrive, driverController);

    // Subsystem Instantiations
    if (HoorayConfig.gimmeConfig().getUsesPhotonVision()) {
      visionSubsystem = new PhotonVisionSubsystem();
      Shuffleboard.getTab("Config").add("Camera", "Photon");
    } else {
      visionSubsystem = new LimlihSubsystem(checkLimelightCommand);
      Shuffleboard.getTab("Config").add("Camera", "Limlih");
    }
    shootSubsystem = new ShootSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    indexSubsystem = new IndexSubsystem();
    armAngleSubsystem = new ArmAngleSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    lineBreakSensorSubsystem = new LineBreakSensorSubsystem();
    poseEstimationSubsystem =
        new PoseEstimationSubsystem(drivetrain, visionSubsystem, armAngleSubsystem);
    loggingSubsystem =
        new LoggingSubsystem(
            armAngleSubsystem,
            elevatorSubsystem,
            indexSubsystem,
            intakeSubsystem,
            lineBreakSensorSubsystem,
            poseEstimationSubsystem,
            shootSubsystem);
    this.lightSubsystem = lightSubsystem;

    // commands for auto
    NamedCommands.registerCommand("rotatie", new PPCenterOnTarget(visionSubsystem));
    NamedCommands.registerCommand("stop", new InstantCommand(() -> drivetrain.stop()));
    NamedCommands.registerCommand(
        "closeShot", new AutoSideShotCommand(shootSubsystem, indexSubsystem));
    NamedCommands.registerCommand(
        "revenald", new InstantCommand(() -> shootSubsystem.changeSetpoint(2000)));
    NamedCommands.registerCommand(
        "speakershoot",
        CommandGroups.autoShoot(
                shootSubsystem,
                indexSubsystem,
                visionSubsystem,
                driverController,
                armAngleSubsystem)
            .withTimeout(2.75));
    NamedCommands.registerCommand(
        "intake",
        CommandGroups.intakeWithLineBreakSensor(
            intakeSubsystem, indexSubsystem, lineBreakSensorSubsystem, armAngleSubsystem));
    alliance = Shuffleboard.getTab("Config").add("Alliance", "aaaanoalliance").getEntry();

    // Command Instantiations
    exampleCommand = new ExampleCommand();
    resetOdometryCommandForward =
        new ResetOdometryCommand(
            new Pose2d(new Translation2d(), new Rotation2d(Math.PI)), drivetrain);
    resetOdometryCommandBackward =
        new ResetOdometryCommand(new Pose2d(new Translation2d(), new Rotation2d(0.0)), drivetrain);
    changeFieldOrientCommand = new ChangeFieldOrientCommand(m_drive);
    centerOnTargetCommand =
        new CenterOnTargetCommand(
            visionSubsystem,
            m_robotDrive,
            AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
            driverController);
    shootCommand = new ShootCommand(shootSubsystem);
    shootAmpCommand = new ShootAmpCommand(shootSubsystem, indexSubsystem);
    elevatorToAmpCommand = new ElevatorToAmpCommand(elevatorSubsystem);
    autoZero = new AutoZero(elevatorSubsystem, armAngleSubsystem);
    shotReverseCommand = new ShotReverseCommand(shootSubsystem);
    toggleIntakeCommand =
        new ToggleIntakeCommand(
            new IntakeSensorCommand(intakeSubsystem, lineBreakSensorSubsystem),
            new IndexSensorCommand(lineBreakSensorSubsystem, indexSubsystem),
            new IndexReverseForShotCommand(lineBreakSensorSubsystem, indexSubsystem),
            armAngleSubsystem);

    elevatorManualCommand =
        new ElevatorManualCommand(
            elevatorSubsystem,
            () -> driverController.getLeftTriggerAxis(),
            () -> driverController.getRightTriggerAxis());
    limDriveSetCommand =
        new LimDriveSetCommand(visionSubsystem, drivetrain, poseEstimationSubsystem);
    // shootSubsystem.setDefaultCommand(shuffleBoardShootCommand);
    driveToTargetCommand = new DriveToTargetCommand(drivetrain, visionSubsystem, 4, -3);

    // armAngleSubsystem.setDefaultCommand(new ShooterAimCommand(visionSubsystem,
    // armAngleSubsystem));

    // armAngleSubsystem.setDefaultCommand(new ShooterAimCommand(limlihSubsystem,
    // armAngleSubsystem));

    new CommandLoginator();

    lightProgressCommand = new LightProgressCommand(lightSubsystem, 0.75, 125, 200);
    lightBlankCommand = new LightBlankCommand(lightSubsystem);
    lightsOnCommand = new LightsOnCommand(lightSubsystem);
    lightRambowCommand = new LightRambowCommand(lightSubsystem);
    lightFastProgressCommand = new LightFastProgressCommand(lightSubsystem);

    // shootSubsystem.setDefaultCommand(shuffleBoardShootCommand);
    // shootSubsystem.setDefaultCommand(shuffleBoardShootCommand);
    // lightUnderGlowSubsystem.setDefaultCommand(lightBlankCommand);

    // driveToTargetCommand = new DriveToTargetCommand(drivetrain, limlihSubsystem, 4, -3);
    // armAngleSubsystem.setDefaultCommand(new ShooterAimCommand(limlihSubsystem,
    // armAngleSubsystem));

    m_chooser = new SendableChooser<>();
    initializeCamera();
    configureButtonBindings();
    configureAutoChooser(drivetrain);
  }

  /** Creates and establishes camera streams for the shuffleboard ~Ben */
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
        poseEstimationSubsystem::setInitialPose,
        m_robotDrive::getChassisSpeed,
        m_robotDrive::setModuleStates,
        new HolonomicPathFollowerConfig(
            HoorayConfig.gimmeConfig().getkTranslationController(),
            HoorayConfig.gimmeConfig().getkThetaController(),
            Constants.AutoConstants.kMaxSpeed,
            Math.sqrt(
                    Math.pow(Constants.DriveConstants.kWheelBaseWidth, 2)
                        + Math.pow(Constants.DriveConstants.kWheelBaseLength, 2))
                / 2,
            new ReplanningConfig(false, false)),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          throw new RuntimeException();
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
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Controller
    driverController.rightTrigger().whileTrue(elevatorManualCommand);
    driverController.leftTrigger().whileTrue(elevatorManualCommand);

    driverController.rightBumper().whileTrue(new ArmUpCommand(armAngleSubsystem));
    driverController.leftBumper().whileTrue(new ArmDownCommand(armAngleSubsystem));

    driverController
        .start()
        .onTrue(
            new CenterOnTargetCommand(
                    visionSubsystem,
                    m_robotDrive,
                    AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
                    driverController)
                .withTimeout(1.5));
    driverController.back().onTrue(changeFieldOrientCommand);

    driverController.a().onTrue(toggleIntakeCommand);

    driverController.b().whileTrue(CommandGroups.outakeFull(intakeSubsystem, indexSubsystem));
    driverController
        .x()
        .onTrue(
            CommandGroups.teleOpShoot(
                shootSubsystem,
                indexSubsystem,
                m_robotDrive,
                visionSubsystem,
                driverController,
                armAngleSubsystem));

    // driverController.b().whileTrue(new IndexCommand(indexSubsystem));
    // driverController.x().whileTrue(new ShuffleBoardShootCommand(shootSubsystem));
    driverController.y().onTrue(new ShootAmpCommand(shootSubsystem, indexSubsystem));

    driverController
        .povUp()
        .onTrue(
            CommandGroups.elevatorAndAngleToAmp(
                shootSubsystem, indexSubsystem, armAngleSubsystem, elevatorSubsystem));
    driverController
        .povRight()
        .onTrue(CommandGroups.FullZeroCommand(elevatorSubsystem, armAngleSubsystem));
    driverController
        .povLeft()
        .onTrue(CommandGroups.armToHorizonalCommandGroup(armAngleSubsystem, elevatorSubsystem));
    driverController
        .povDown()
        .onTrue(CommandGroups.armToIntakeCommandGroup(armAngleSubsystem, elevatorSubsystem));

    driverController.rightStick().whileTrue(exampleCommand);
    driverController.leftStick().whileTrue(resetOdometryCommandForward); // field orient

    // Operator Controller
    operatorController.rightTrigger().whileTrue(elevatorManualCommand);
    operatorController.leftTrigger().whileTrue(elevatorManualCommand);

    operatorController.rightBumper().whileTrue(new MoveArmCommand(armAngleSubsystem, 0.01));
    operatorController.leftBumper().whileTrue(new MoveArmCommand(armAngleSubsystem, -0.01));

    operatorController
        .start()
        .whileTrue(
            new CenterOnTargetCommand(
                visionSubsystem,
                m_robotDrive,
                AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
                driverController));
    operatorController.back().onTrue(changeFieldOrientCommand);

    operatorController.a().onTrue(toggleIntakeCommand);
    operatorController.b().whileTrue(new IndexCommand(indexSubsystem));
    operatorController.x().whileTrue(new ShuffleBoardShootCommand(shootSubsystem));
    // operatorController.y().whileTrue(new ElevatorUpCommand(elevatorSubsystem));// hi jonny was
    // here

    operatorController
        .povUp()
        .onTrue(
            CommandGroups.elevatorAndAngleToAmp(
                shootSubsystem, indexSubsystem, armAngleSubsystem, elevatorSubsystem));
    operatorController
        .povRight()
        .onTrue(CommandGroups.FullZeroCommand(elevatorSubsystem, armAngleSubsystem));
    operatorController
        .povLeft()
        .onTrue(new DriveByController(m_robotDrive, operatorController, false));
    operatorController
        .povDown()
        .onTrue(CommandGroups.armToIntakeCommandGroup(armAngleSubsystem, elevatorSubsystem));
  }

  // jonathan was here today 2/3/2023
  /* Pulls autos and configures the chooser */
  // SwerveAutoBuilder swerveAutoBuilder;
  Map<Command, String> autoName = new HashMap<Command, String>();

  private void configureAutoChooser(Drivetrain drivetrain) {
    configureAutoBuilder();

    File pathPlannerDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner");
    pathPlannerDirectory = new File(pathPlannerDirectory, "autos");

    for (File pathFile : pathPlannerDirectory.listFiles()) {

      if (pathFile.isFile() && pathFile.getName().endsWith(".auto")) {

        String name = pathFile.getName().replace(".auto", "");
        Command pathCommand = new PathPlannerAuto(name);
        Command autoCommand =
            new SequentialCommandGroup(
                CommandGroups.intakeWithLineBreakSensor(
                    intakeSubsystem, indexSubsystem, lineBreakSensorSubsystem, armAngleSubsystem),
                pathCommand,
                new InstantCommand(drivetrain::stop));
        m_chooser.addOption(name, autoCommand);

        autoName.put(autoCommand, name);
      }

      SysIdRoutine sysIdRoutine =
          new SysIdRoutine(
              new SysIdRoutine.Config(),
              new SysIdRoutine.Mechanism(
                  shootSubsystem::setVoltage, shootSubsystem::getData, shootSubsystem));
      m_chooser.addOption(
          "yes",
          new SequentialCommandGroup(
              sysIdRoutine.dynamic(Direction.kForward),
              new WaitCommand(5),
              sysIdRoutine.dynamic(Direction.kReverse),
              new WaitCommand(5),
              sysIdRoutine.quasistatic(Direction.kForward),
              new WaitCommand(5),
              sysIdRoutine.quasistatic(Direction.kReverse)));
    }
    // m_chooser.addOption("Example Path", new PathPlannerAuto("New Auto"));

    Shuffleboard.getTab("RobotData").add("SelectAuto", m_chooser).withSize(3, 2).withPosition(0, 0);
  }

  public void robotInit() {
    // new AutoZero(elevatorSubsystem, armAngleSubsystem).schedule();
    limDriveSetCommand.schedule();
  }

  public void autonomousInit() {

    limDriveSetCommand.schedule();
  }

  public void teleopInit() {
    m_robotDrive.setDefaultCommand(m_drive);
    limDriveSetCommand.schedule();
    // autoZero.schedule();
  }

  public void autonomousPeriodic() {}

  public void teleopPeriodic() {}

  /**
   * @return Selected Auto
   */
  public Command getAuto() {
    return m_chooser.getSelected();
  }

  public void configureTestMode() {

    m_robotDrive.setDefaultCommand(new CoastCommand(m_robotDrive));
  }

  public String getAutoName(Command command) {
    return autoName.containsKey(command) ? autoName.get(command) : "Nothing?????/?///?";
  }
}
