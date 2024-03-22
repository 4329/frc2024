package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.commands.driveCommands.CenterOnTargetCommandIndefinite;
import frc.robot.commands.shootCommands.ShooterAimCommandIndefinite;
import frc.robot.commands.shootCommands.ShooterShotCommand;
import frc.robot.commands.shootCommands.ShotRevCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.ArmAngle;

public class TeleopShootCommand extends SequentialCommandGroup {
    public TeleopShootCommand(
            ShootSubsystem shootSubsystem,
            IndexSubsystem indexSubsystem,
            Drivetrain drivetrain,
            VisionSubsystem visionSubsystem,
            CommandXboxController commandXboxController,
            ArmAngleSubsystem armAngleSubsystem) {

        super(
                new ParallelRaceGroup(
                        new ShotRevCommand(shootSubsystem, visionSubsystem).withTimeout(3),
                        new CenterOnTargetCommandIndefinite(
                                visionSubsystem,
                                drivetrain,
                                AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
                                commandXboxController),
                        new ShooterAimCommandIndefinite(visionSubsystem, armAngleSubsystem)),
                new ParallelRaceGroup(
                        new ShooterShotCommand(shootSubsystem, indexSubsystem, visionSubsystem),
                        new CenterOnTargetCommandIndefinite(
                                visionSubsystem,
                                drivetrain,
                                AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
                                commandXboxController),
                        new ShooterAimCommandIndefinite(visionSubsystem, armAngleSubsystem)),
                new ArmCommand(armAngleSubsystem, ArmAngle.INTAKE));
    }
}
