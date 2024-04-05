package frc.robot.commands.shootCommands;

// import java.lang.reflect.Field;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.ArmAngleSubsystem;
// import frc.robot.subsystems.IndexSubsystem;
// import frc.robot.subsystems.LimlihSubsystem;
// import frc.robot.subsystems.ShootSubsystem;
// import frc.robot.subsystems.swerve.Drivetrain;
// import frc.robot.utilities.AprilTagUtil;
// import frc.robot.utilities.FieldRelativeAccel;
// import frc.robot.utilities.FieldRelativeSpeed;
// import frc.robot.utilities.MathUtils;

// public class MoveAndShootCommand extends Command {

//     XboxController xboxController;
//     ShootSubsystem shootSubsystem;
//     IndexSubsystem indexSubsystem;
//     LimlihSubsystem limlihSubsystem;
//     Drivetrain drivetrain;
//     ArmAngleSubsystem armAngleSubsystem;

//     Timer timer = new Timer();
//     PIDController swervepid = new PIDController(2.0, 0, 0.1);
//     // NetworkTableEntry tGEntry = Shuffleboard.getTab("RobotData").add("tG", 1).getEntry();
//     // NetworkTableEntry tREntry = Shuffleboard.getTab("RobotData").add("tR", 1).getEntry();

//     public MoveAndShootCommand(ShootSubsystem shootSubsystem, Drivetrain drivetrain,
// IndexSubsystem indexSubsystem, XboxController xboxController, LimlihSubsystem limlihSubsystem,
// ArmAngleSubsystem armAngleSubsystem) {

//         this.shootSubsystem = shootSubsystem;
//         this.drivetrain = drivetrain;
//         this.indexSubsystem = indexSubsystem;
//         this.armAngleSubsystem = armAngleSubsystem;
//         // this.turret = turret;
//         this.xboxController = xboxController;
//         this.limlihSubsystem = limlihSubsystem;
//         addRequirements(shootSubsystem, drivetrain, indexSubsystem, limlihSubsystem);

//         swervepid.enableContinuousInput(0, 2 * Math.PI);
//     }

//     @Override
//     public void initialize() {

//         timer.reset();
//         timer.start();
//     }

//     @Override
//     public void execute() {

//         double currentTime = timer.get();
//         double RobotSpeedx = drivetrain.getChassisSpeed().vxMetersPerSecond;
//         double RobotSpeedy = drivetrain.getChassisSpeed().vyMetersPerSecond;
//         double Velocity = Math.pow((Math.pow(RobotSpeedx, 2) + Math.pow(RobotSpeedy, 2)), 0.5);
//         FieldRelativeAccel accel = drivetrain.getRelativeAccel();
//         Pose3d pose3d =
// limlihSubsystem.getTargetPoseInRobotSpace(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker());
//         double dist = pose3d.getZ();
//         double translationX = pose3d.getX();
//         double translationY = pose3d.getY();
//         Translation2d movingGoal = new Translation2d();

//         for (int i = 0; i < 5; i++) {

//             double virtualGoalx = translationX -
//                     (RobotSpeedx + accel.ax * 0.1);

//             double virtualGoalY = translationY -
//                     (RobotSpeedy + accel.ay * 0.1);

//             Translation2d testGoal = new Translation2d(virtualGoalx, virtualGoalY);
//     RobotSpeedy

//             double ExpectedPose3d = pose3d + (Velocity + Math.pow(Velocity, 2) / 0.5);
//             double newShotTime = Constants.TuningConstants.kTimeTable
//                     .getOutput(MathUtils.MetersToInches(robotToTestGoal.getDistance(new
// Translation2d())));

//             if (Math.abs(newShotTime - shotTime) <= 0.10) {

//                 i = 4;
//             }

//             if (i == 4) {

//                 movingGoal = testGoal;
//             } else {

//                 shotTime = newShotTime;
//             }
//         }

//         Translation2d robotToMovingGoal =
// movingGoal.minus(drivetrain.getPose().getTranslation());

//         double newDistance = MathUtils.MetersToInches(robotToMovingGoal.getDistance(new
// Translation2d()));

//         if (xboxController.getLeftTriggerAxis() > 0.5) {

//
// hood.setEncoderPosition(Constants.TuningConstants.m_hoodTable.getOutput(newDistance));

//             if (hood.atSetpoint()) {
//                 storageIntake.storageIntakeInSlow();
//                 shooterFeedSubsytem.shooterFeedFire();
//             } else {
//                 storageIntake.storageIntakeStop();
//                 shooterFeedSubsytem.shooterFeedStop();
//             }
//         } else {
//             hood.setPosition(HoodPosition.OPEN);
//             storageIntake.storageIntakeStop();
//             shooterFeedSubsytem.shooterFeedStop();
//         }

//         shooter.shoot(Constants.TuningConstants.m_rpmTable.getOutput(newDistance));

//         double targetAngle = Math.atan2(robotToMovingGoal.getY(), robotToMovingGoal.getX()) +
// Math.PI;
//         targetAngle = MathUtils.toUnitCircAngle(targetAngle);
//         double currentAngle = MathUtils.toUnitCircAngle(drivetrain.getGyro().getRadians());

//         double pidOutput = swervepid.calculate(currentAngle, targetAngle);

//         drivetrain.drive(
//                 -inputTransform(xboxController.getLeftY())
//                         * Constants.DriveConstants.kMaxSpeedMetersPerSecond,
//                 -inputTransform(xboxController.getLeftX())
//                         * Constants.DriveConstants.kMaxSpeedMetersPerSecond,
//                 pidOutput,
//                 true);

//         if (currentTime > 0.250 && TurretSubsystem.targetVisible() &&
// TurretSubsystem.getDistanceFromTarget() >= 85.0
//                 && hood.getEncoderPos() < 4.0) {
//             // double dL = TurretSubsystem.getDistanceFromTarget() * 0.0254;
//             double dL = MathUtils.inchesToMeters(TurretSubsystem.getDistanceFromTarget());
//             double tR = drivetrain.getGyro().getRadians();
//             double tT = Math.PI;
//             double tL = -1.0 * TurretSubsystem.getTx();

//             Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, Constants.ShooterConstants.goalPos);

//             drivetrain.setPose(pose);
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {

//         shooter.holdFire();
//         hood.setPosition(HoodPosition.OPEN);
//         storageIntake.storageIntakeStop();
//         shooterFeedSubsytem.shooterFeedStop();
//     }

//     private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d
// goal) {

//         double tG = tR + tT + Math.toRadians(tL);
//         double rX = goal.getX() - dL * Math.cos(tG);
//         double rY = goal.getY() - dL * Math.sin(tG);

//         tREntry.setDouble(tR);
//         tGEntry.setDouble(tG);

//         return new Pose2d(rX, rY, new Rotation2d(-tR));
//     }

//     private double inputTransform(double input) {

//         return MathUtils.singedSquare(MathUtils.applyDeadband(input));
//     }

// }
