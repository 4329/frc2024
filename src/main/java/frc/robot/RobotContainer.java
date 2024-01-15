package frc.robot;

import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CommandGroups;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShuffleBoardShootCommand;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.commands.armCommands.ArmDownCommand;
import frc.robot.commands.armCommands.ArmUpCommand;
import frc.robot.commands.drive.CenterOnTargetCommand;
import frc.robot.commands.drive.ChangeFieldOrientCommand;
import frc.robot.commands.drive.CoastCommand;
import frc.robot.commands.drive.DriveByController;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.drive.ResetOdometryTargetSpaceCommand;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShootSubsystem;
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
  private final LimlihSubsystem limlihSubsystem;
  private final ShootSubsystem shootSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final IndexSubsystem indexSubsystem;
  private final ArmAngleSubsystem armAngleSubsystem;
  private final PoseEstimationSubsystem poseEstimationSubsystem;  
  private final ElevatorSubsystem elevatorSubsystem;

  // Command Declarations
  private final ExampleCommand exampleCommand;
  private final ResetOdometryCommand resetOdometryCommandForward;
  private final ResetOdometryCommand resetOdometryCommandBackward;
  private final ChangeFieldOrientCommand changeFieldOrientCommand;
  private final ShuffleBoardShootCommand shuffleBoardShootCommand;
  

  private final CenterOnTargetCommand centerOnTargetCommand;
  private final ShootCommand shootCommand;
  private final DriveToTargetCommand driveToTargetCommand;



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
    limlihSubsystem = new LimlihSubsystem();
    shootSubsystem = new ShootSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    indexSubsystem = new IndexSubsystem();
    armAngleSubsystem = new ArmAngleSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    poseEstimationSubsystem = new PoseEstimationSubsystem(drivetrain, limlihSubsystem);

    // Command Instantiations
    exampleCommand = new ExampleCommand();
    resetOdometryCommandForward = new ResetOdometryCommand(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)),
    drivetrain);
    resetOdometryCommandBackward = new ResetOdometryCommand(new Pose2d(new Translation2d(), new Rotation2d(0.0)),
    drivetrain);
    changeFieldOrientCommand = new ChangeFieldOrientCommand(m_drive);
    centerOnTargetCommand = new CenterOnTargetCommand(limlihSubsystem, m_robotDrive, 4, driverController);
    shootCommand = new ShootCommand(shootSubsystem);
    shuffleBoardShootCommand = new ShuffleBoardShootCommand(shootSubsystem);

    
    shootSubsystem.setDefaultCommand(shuffleBoardShootCommand);
    driveToTargetCommand = new DriveToTargetCommand(drivetrain, limlihSubsystem, 4, -1.5);
    
    m_chooser = new SendableChooser<>();
    initializeCamera();
    configureButtonBindings();
    configureAutoChooser(drivetrain);
  }

  /**
   * Creates and establishes camera streams for the shuffleboard ~Ben
   */
  private void initializeCamera() {

    CameraServer.startAutomaticCapture();
    // VideoSource[] enumerateSources = VideoSource.enumerateSources();

    // if (enumerateSources.length > 0 &&
    // enumerateSources[0].getName().contains("USB")) {
    // Shuffleboard.getTab("RobotData").add("Camera",
    // enumerateSources[0]).withPosition(5, 0).withSize(3, 3)
    // .withWidget(BuiltInWidgets.kCameraStream);
    // }

    HttpCamera limelight = new HttpCamera("Limelight", HoorayConfig.gimmeConfig().getLimelighturl());
    System.out.println(HoorayConfig.gimmeConfig().getLimelighturl());
    CameraServer.startAutomaticCapture(limelight);

    // Shuffleboard.getTab("RobotData").add("Limelight Camera", limelight).withPosition(2, 0).withSize(2, 2)
    //     .withWidget(BuiltInWidgets.kCameraStream);
  }

  /* Autonomous :D */
  private Map<String, Command> createEventMap() {
    Map<String, Command> eventMap = new HashMap<>();
    eventMap.put("Example Command", new ExampleCommand());
    return eventMap;
  }

  private void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
        m_robotDrive::getPose,
        m_robotDrive::resetOdometry,
        m_robotDrive::getChassisSpeed,
        m_robotDrive::setModuleStates,
        new HolonomicPathFollowerConfig(
            new PIDConstants(Constants.AutoConstants.kPXController),
            new PIDConstants(Constants.AutoConstants.kPThetaController),
            Constants.AutoConstants.kMaxSpeed,
            0.4,
            new ReplanningConfig()
        ),
        () -> {
          Optional<Alliance> alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get().equals(DriverStation.Alliance.Red) : false;
        },
        m_robotDrive
    );
  }

//  private SwerveAutoBuilder createAutoBuilder() {
//
//    SwerveAutoBuilder autoBuilder = new GimmeSwerve(
//
//      m_robotDrive::getPose, // Pose2d supplier
//      m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
//      Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
//      new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation
//                                                                         // error (used to create the X and Y PID
//                                                                         // controllers)
//      new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation
//    // error (used to create the rotation
//
//      m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
//      createEventMap(),
//      true, // Should the path be automatically mirrored depending on alliance color.
//             // Optional, defaults to true
//      m_robotDrive // The drive subsystem. Used to properly set the requirements of path following
//                    // commands
//    );
//
//    return autoBuilder;
//  }

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
    driverController.rightTrigger().whileTrue(exampleCommand);
    driverController.leftTrigger().whileTrue(exampleCommand);
    
    driverController.rightBumper().whileTrue(exampleCommand);
    driverController.leftBumper().whileTrue(exampleCommand);

    driverController.start().onTrue(exampleCommand);
    driverController.back().onTrue(changeFieldOrientCommand);

    driverController.a().whileTrue(shootCommand);
    driverController.b().onTrue(exampleCommand);
    driverController.x().whileTrue(new SequentialCommandGroup(new ResetOdometryTargetSpaceCommand(limlihSubsystem, m_robotDrive, 4), driveToTargetCommand));
    driverController.y().whileTrue(centerOnTargetCommand);

    driverController.povUp().onTrue(exampleCommand);
    driverController.povRight().onTrue(exampleCommand);
    driverController.povLeft().onTrue(exampleCommand);
    driverController.povDown().onTrue(exampleCommand);

    driverController.rightStick().whileTrue(exampleCommand);
    driverController.leftStick().whileTrue(resetOdometryCommandForward); //field orient
    
    // Operator Controller
    operatorController.rightTrigger().whileTrue(exampleCommand);
    operatorController.leftTrigger().whileTrue(exampleCommand);
  
    operatorController.rightBumper().whileTrue(exampleCommand); //arm up
    operatorController.leftBumper().whileTrue(exampleCommand); //arm down

    operatorController.start().whileTrue(exampleCommand); //to april tag or conecubetoggle
    operatorController.back().onTrue(changeFieldOrientCommand);

    operatorController.a().whileTrue(CommandGroups.intakeFull(intakeSubsystem, indexSubsystem));
    operatorController.b().whileTrue(CommandGroups.outakeFull(intakeSubsystem, indexSubsystem));
    operatorController.x().whileTrue(new ArmUpCommand(armAngleSubsystem));
    operatorController.y().whileTrue(new ArmDownCommand(armAngleSubsystem));//hi jonny was here

    operatorController.povUp().onTrue(new ArmAngleCommand(armAngleSubsystem, ArmAngle.ZERO));
    operatorController.povRight().onTrue(exampleCommand);
    operatorController.povLeft().onTrue(exampleCommand);
    operatorController.povDown().onTrue(new ArmAngleCommand(armAngleSubsystem,ArmAngle.INTAKE));
  }

  // jonathan was here today 2/3/2023
  /* Pulls autos and configures the chooser */
  //SwerveAutoBuilder swerveAutoBuilder;

  private void configureAutoChooser(Drivetrain drivetrain) {
    configureAutoBuilder();

    //swerveAutoBuilder = createAutoBuilder();
    File pathPlannerDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner");
    pathPlannerDirectory = new File(pathPlannerDirectory, "autos");

    for (File pathFile : pathPlannerDirectory.listFiles()) {

//      System.out.println(pathFile);

      if (pathFile.isFile() && pathFile.getName().endsWith(".auto")) {

        String name = pathFile.getName().replace(".auto", "");

//        List<PathConstraints> constraints = getPathConstraints(name);

//        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(name, constraints);

        Command pathCommand =  new PathPlannerAuto(name);
        if (name.endsWith("BalanceAuto")) {
//          m_chooser.addOption(name, new SequentialCommandGroup(pathCommand, new BalanceCommand(m_robotDrive, balanceSubsystem).withTimeout(12)));
        } else {
          m_chooser.addOption(name, pathCommand);
        }
      }
    }
//    m_chooser.addOption("Example Path", new PathPlannerAuto("New Auto"));

    Shuffleboard.getTab("RobotData").add("SelectAuto", m_chooser).withSize(3, 2).withPosition(0, 0);
  }
      

  public void autonomousInit() {

  }

  public void teleopInit() {
    m_robotDrive.setDefaultCommand(m_drive);
  }

  public void autonomousPeriodic() {

  }

  public void teleopPeriodic() {

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