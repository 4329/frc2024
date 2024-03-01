package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Model.PoseEstimationLog;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.FieldRelativeAccel;
import frc.robot.utilities.FieldRelativeSpeed;
import frc.robot.utilities.MathUtils;

public class MoveAndShoot extends Command {
    private Drivetrain drivetrain;
    private VisionSubsystem visionSubsystem;
    private PoseEstimationSubsystem poseEstimationSubsystem;
    private ShootSubsystem shootSubsystem; 
    private ArmAngleSubsystem armAngleSubsystem;
    private XboxController xboxController;

    private PIDController rotPID;
    private Timer timer;
    private int tagID;

    public MoveAndShoot(Drivetrain drivetrain, VisionSubsystem visionSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem, ShootSubsystem shootSubsystem,
            ArmAngleSubsystem armAngleSubsystem, int tagID, XboxController xboxController) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.poseEstimationSubsystem = poseEstimationSubsystem;
        this.shootSubsystem = shootSubsystem;
        this.armAngleSubsystem = armAngleSubsystem;
        this.tagID = tagID;
        this.xboxController = xboxController;

        rotPID = new PIDController(0, 0, 0);
        rotPID.enableContinuousInput(0, 2 * Math.PI);

        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double curTime = timer.get();
        FieldRelativeSpeed fieldRelativeSpeed = drivetrain.getRelativeSpeed();
        FieldRelativeAccel fieldRelativeAccel = drivetrain.getRelativeAccel();
        Translation2d goal = visionSubsystem.field.getTagPose(tagID).get().getTranslation().toTranslation2d();
        Translation2d robotToGoal = goal.minus(poseEstimationSubsystem.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d());
        double shotTime = 9;
        Translation2d movingGoal = new Translation2d();

        for (int i = 0; i < 5; i++) {
            double virtualGoalX = goal.getX() - shotTime * (fieldRelativeSpeed.vx + fieldRelativeAccel.ax * 0.1);
            double virtualGoalY = goal.getY() - shotTime * (fieldRelativeSpeed.vy + fieldRelativeAccel.ay * 0.1);

            Translation2d testGoal = new Translation2d(virtualGoalX, virtualGoalY);
            Translation2d robotToTestGoal = testGoal.minus(poseEstimationSubsystem.getPose().getTranslation());

            double newShotTime = 0;

            if (Math.abs(newShotTime - shotTime) <= 0.1) {
                i = 4;
            }

            if (i == 4) {
                movingGoal = testGoal;
            } else {
                shotTime = newShotTime;
            }
        }

        Translation2d robotToMovingGoal = movingGoal.minus(poseEstimationSubsystem.getPose().getTranslation());
        double newDst = robotToMovingGoal.getDistance(new Translation2d());

        double targetAngle = Math.atan2(robotToMovingGoal.getY(), robotToMovingGoal.getY()) + Math.PI;
        targetAngle = MathUtils.toUnitCircAngle(targetAngle);
        double curAngle = MathUtils.toUnitCircAngle(poseEstimationSubsystem.getPose().getRotation().getRadians());

        double pidOut = rotPID.calculate(curAngle, targetAngle);

        drivetrain.drive(
            -inputTransform(xboxController.getLeftY()), 
            -inputTransform(xboxController.getLeftX()), 
            pidOut,
            true);
    }

    
    private double inputTransform(double input) {
        return MathUtils.singedSquare(MathUtils.applyDeadband(input));
    }
}