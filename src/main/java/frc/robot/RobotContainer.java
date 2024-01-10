package frc.robot;

import java.util.HashMap;
import java.util.Map;

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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drive.ChangeFieldOrientCommand;
import frc.robot.commands.drive.CoastCommand;
import frc.robot.commands.drive.DriveByController;
import frc.robot.commands.drive.ResetOdometryCommand;
import frc.robot.subsystems.swerve.Drivetrain;
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

  private final ExampleCommand exampleCommand;
  private final ResetOdometryCommand resetOdometryCommandForward;
  private final ResetOdometryCommand resetOdometryCommandBackward;
  private final ChangeFieldOrientCommand changeFieldOrientCommand;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @param drivetrain
   */

  public RobotContainer(Drivetrain drivetrain) {

    // pid = Shuffleboard.getTab("yes").add("name", 0).withWidget(BuiltInWidgets.kGraph)
        // .withProperties(Map.of("Automatic bounds", false, "Upper bound", 20)).getEntry();
    m_robotDrive = drivetrain;

    initializeCamera();

    operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
    driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    m_drive = new DriveByController(m_robotDrive, driverController);

    m_chooser = new SendableChooser<>();

    exampleCommand = new ExampleCommand();
    resetOdometryCommandForward = new ResetOdometryCommand(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)),
        drivetrain);
    resetOdometryCommandBackward = new ResetOdometryCommand(new Pose2d(new Translation2d(), new Rotation2d(0.0)),
        drivetrain);
    changeFieldOrientCommand = new ChangeFieldOrientCommand(m_drive);
    //aprilTagMiddleCommand = new CenterOnTargetCommand(limlighSubsystem, m_robotDrive, 1, driverController);
    configureButtonBindings();  /**
                                * Configure the button bindings to commands using configureButtonBindings
                                * function
                                */
    configureAutoChooser(drivetrain);

    // centerOnTargetCommand = new CenterOnTargetCommand(limlighSubsystem, 0, m_robotDrive);
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

  private void ajksfd() {
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
            () -> {return false;},
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

    driverController.a().onTrue(exampleCommand);
    driverController.b().onTrue(exampleCommand);
    driverController.x().onTrue(exampleCommand);
    driverController.y().onTrue(exampleCommand);

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

    operatorController.a().onTrue(exampleCommand);
    operatorController.b().onTrue(exampleCommand);
     operatorController.x().whileTrue(exampleCommand);
    operatorController.y().onTrue(exampleCommand);

    operatorController.povUp().onTrue(exampleCommand);
    operatorController.povRight().onTrue(exampleCommand);
    operatorController.povLeft().onTrue(exampleCommand);
    operatorController.povDown().onTrue(exampleCommand);
  }

  // jonathan was here today 2/3/2023
  /* Pulls autos and configures the chooser */
  //SwerveAutoBuilder swerveAutoBuilder;

  private void configureAutoChooser(Drivetrain drivetrain) {
//
//    //swerveAutoBuilder = createAutoBuilder();
//    File pathPlannerDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner");
//
//    for (File pathFile : pathPlannerDirectory.listFiles()) {
//
//      System.out.println(pathFile);
//
//      if (pathFile.isFile() && pathFile.getName().endsWith(".path")) {
//
//        String name = pathFile.getName().replace(".path", "");
//
//        List<PathConstraints> constraints = getPathConstraints(name);
//
//        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(name, constraints);
//
//        Command pathCommand =  swerveAutoBuilder.fullAuto(trajectories);
//        if (name.endsWith("BalanceAuto")) {
//
//          m_chooser.addOption(name, new SequentialCommandGroup(pathCommand, new BalanceCommand(m_robotDrive, balanceSubsystem).withTimeout(12)));
//        } else {
//
//          m_chooser.addOption(name, pathCommand);
//        }
//
//
//      }
//    }
    ajksfd();
//    Command nooowhy = new PathPlannerAuto("Example Path");
    m_chooser.addOption("Example Path", new PathPlannerAuto("New Auto"));

    Shuffleboard.getTab("RobotData").add("SelectAuto", m_chooser).withSize(3, 2).withPosition(0, 0);
  }

//  private List<PathConstraints> getPathConstraints(String name) {
//
//
//    List<PathConstraints> listConstraints = new ArrayList<>();
//
//    if (name.equalsIgnoreCase("1HighCubeAcrossCenterBalanceAuto")) {
//
//      System.out.println("is constrained");
//      listConstraints.add(new PathConstraints(3.25, 2.5));
//      listConstraints.add(new PathConstraints(3.25, 2.5));
//      listConstraints.add(new PathConstraints(1.75, 1.85));
//      listConstraints.add(new PathConstraints(3, 3));
//
//    } else if (name.equalsIgnoreCase("2PieceOpenAuto")) {
//
//      System.out.println("is constrained");
//      listConstraints.add(new PathConstraints(3.25, 2.5));
//      listConstraints.add(new PathConstraints(2, 1.5));
//      listConstraints.add(new PathConstraints(3.25, 2.5));
//      listConstraints.add(new PathConstraints(3.25, 2.5));
//
//    } else if (name.equalsIgnoreCase("1HighConeAcrossCenterBalanceAuto")) {
//
//      System.out.println("is constrained");
//      listConstraints.add(new PathConstraints(3.25, 2.5));
//      listConstraints.add(new PathConstraints(3.25, 2.5));
//      listConstraints.add(new PathConstraints(1.75, 1.85));
//      listConstraints.add(new PathConstraints(3, 3));
//
//    } else if (name.equalsIgnoreCase("1MidConeAcrossCenterBalanceAuto")) {
//
//      System.out.println("is constrained");
//      listConstraints.add(new PathConstraints(3.25, 2.5));
//      listConstraints.add(new PathConstraints(3.25, 2.5));
//      listConstraints.add(new PathConstraints(1.75, 1.85));
//      listConstraints.add(new PathConstraints(3, 3));
//    } else {
//
//      System.out.println("++++++++++++++++++++++++++++++++++++++++++++");
//      listConstraints.add(new PathConstraints(Constants.AutoConstants.kMaxSpeed, Constants.AutoConstants.kMaxAcceleration));
//    }
//
//    return listConstraints;
//  }
      

  public void autonomousInit() {

  }

  public void teleopInit() {
    m_robotDrive.setDefaultCommand(m_drive);
  }

  public void autonomousPeriodic() {

  }

  public void teleopPeriodic() {

    // if (jsdaklfsd.getBoolean(false)) {

    //   aprilTagMiddleCommand.schedule();
    //   jsdaklfsd.setBoolean(false);
    // }
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