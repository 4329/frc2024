package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.MathUtils;

public class CenterOnTargetCommand extends Command {

    private final LimlihSubsystem limlihSubsystem;
    private final Drivetrain drivetrain;
    private final double targetId;

    private final PIDController rotationPID;
    private CommandXboxController xboxController;
    private Timer timer = new Timer();
    // GenericEntry aprilTag;

    // private boolean usingAprilTag;

    public CenterOnTargetCommand(LimlihSubsystem limlihSubsystem, Drivetrain m_drivetrain, double targetId,
            CommandXboxController xboxController) {

        this.limlihSubsystem = limlihSubsystem;
        this.drivetrain = m_drivetrain;
        this.targetId = targetId;
        this.xboxController = xboxController;

        rotationPID = new PIDController(0.00002, 0, 0);
        addRequirements(limlihSubsystem, m_drivetrain);
        // aprilTag = Shuffleboard.getTab("RobotData").add("painhahah", false).getEntry();
    }

    @Override

    public void initialize() {

        timer.reset();
        timer.start();

        // if (limlihSubsystem.getPipeline() == 0) {

        // usingAprilTag = true;

        rotationPID.setP(0.05);
        rotationPID.setTolerance(0);
        rotationPID.setSetpoint(0);
        // } else {

        // usingAprilTag = false;

        // centerPID.setP(0);

        // centerPID.setTolerance(10000);

        // centerPID.setSetpoint(0);

        // forwardPID.setP(0);

        // forwardPID.setTolerance(10000);

        // forwardPID.setSetpoint(0);

        // rotationPID.setP(0.000005);

        // rotationPID.setTolerance(0);

        // rotationPID.setSetpoint(0);
        // }

    }

    @Override

    public void execute() {

        // double centerCalc = 0;
        // double forwardCalc = 0;
        double rotationCalc = 0;
        if ((limlihSubsystem.getTargetId() == targetId)) {

            // aprilTag.setBoolean(true);

            // centerCalc = centerPID.calculate(limlihSubsystem.getTargetx());
            // forwardCalc = forwardPID.calculate(limlihSubsystem.getCalculatedPoseZ());
            rotationCalc = rotationPID.calculate(limlihSubsystem.getTargetX());

            if (rotationCalc > Constants.DriveConstants.kMaxAngularSpeed)
                rotationCalc = Constants.DriveConstants.kMaxAngularSpeed;
            else if (rotationCalc < -Constants.DriveConstants.kMaxAngularSpeed)
                rotationCalc = -Constants.DriveConstants.kMaxAngularSpeed;
            else if (rotationPID.atSetpoint())
                rotationCalc = 0;

            double adjTranslation = ((Constants.DriveConstants.kMaxAngularSpeed - Math.abs(rotationCalc))
                    / Constants.DriveConstants.kMaxAngularSpeed) * 0.5;

            drivetrain.drive(
                    -inputTransform(xboxController.getLeftY())
                            * (Constants.DriveConstants.kMaxSpeedMetersPerSecond * adjTranslation),
                    -inputTransform(xboxController.getLeftX())
                            * (Constants.DriveConstants.kMaxSpeedMetersPerSecond * adjTranslation),
                    rotationCalc,
                    true);
            // System.out.println(rotationCalc);
        } // else if (limlihSubsystem.targetVisible()) {

        // aprilTag.setBoolean(false);

        // rotationCalc = rotationPID.calculate(limlihSubsystem.getTargetx(), 0);
        // forwardCalc = forwardPID.calculate(0);
        // centerCalc = centerPID.calculate(0);

        // if (rotationCalc > Constants.DriveConstants.kMaxAngularSpeed)
        // rotationCalc = Constants.DriveConstants.kMaxAngularSpeed;
        // else if (rotationCalc < -Constants.DriveConstants.kMaxAngularSpeed)
        // rotationCalc = -Constants.DriveConstants.kMaxAngularSpeed;
        // else if (rotationPID.atSetpoint())
        // rotationCalc = 0;

        // double adjTranslation = ((Constants.DriveConstants.kMaxAngularSpeed -
        // Math.abs(rotationCalc))
        // / Constants.DriveConstants.kMaxAngularSpeed) * 0.5;

        // drivetrain.drive(
        // -inputTransform(xboxController.getLeftY()) *
        // (Constants.DriveConstants.kMaxSpeedMetersPerSecond * adjTranslation),
        // -inputTransform(xboxController.getLeftX()) *
        // (Constants.DriveConstants.kMaxSpeedMetersPerSecond * adjTranslation),
        // rotationCalc,
        // true
        // );

        // }

        // if (!rotationPID.atSetpoint()) {

        //     drivetrain.unlock();
        //     drivetrain.drive(0, 0, rotationCalc, false);
        // } else {
        //     drivetrain.lock();
        // }

    }

    @Override

    public boolean isFinished() {
        return false;
    }

    @Override

    public void end(boolean interrupted) {
        drivetrain.unlock();

        // aprilTag.setBoolean(false);
    }

    private double inputTransform(double input) {

        return MathUtils.singedSquare(MathUtils.applyDeadband(input));
    }

}
