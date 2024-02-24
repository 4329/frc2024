package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.armCommands.ShooterAimCommand;
import frc.robot.commands.drive.CenterOnTargetCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.AprilTagUtil;

public class CommandGroups {

        public static Command intakeFull(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {

                return new ParallelCommandGroup(

                                new IntakeCommand(intakeSubsystem),
                                new IndexCommand(indexSubsystem)

                );

        }

        public static Command intakeWithLineBreakSensor(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem,
                        LineBreakSensorSubsystem lineBreakSensorSubsystem) {

                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new IntakeSensorCommand(intakeSubsystem, lineBreakSensorSubsystem),
                                                new IndexSensorCommand(lineBreakSensorSubsystem, indexSubsystem)

                                ),
                                new IndexReverseForShotCommand(lineBreakSensorSubsystem, indexSubsystem));

        }

        public static Command outakeFull(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {

                return new ParallelCommandGroup(

                                new OutakeCommand(intakeSubsystem),
                                new OutdexCommand(indexSubsystem)

                );

        }

        public static Command releaseToShoot(ShootSubsystem shootSubsystem, IndexSubsystem indexSubsystem) {

                return new ParallelCommandGroup(
                                new IndexCommand(indexSubsystem).withTimeout(3),
                                new ShootCommand(shootSubsystem).withTimeout(3));

        }

        public static Command aimAndShoot(ShootSubsystem shootSubsystem, Drivetrain m_robotDrive,
                        IndexSubsystem indexSubsystem, VisionSubsystem visionSubsystem,
                        CommandXboxController driverController,
                        ArmAngleSubsystem armAngleSubsystem) {

                return new SequentialCommandGroup(

                                new ParallelCommandGroup(
                                                new CenterOnTargetCommand(visionSubsystem, m_robotDrive,
                                                                AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
                                                                driverController).withTimeout(1.25),
                                                new ShooterAimCommand(visionSubsystem, armAngleSubsystem)
                                                                .withTimeout(1.25)

                                ),

                                new WaitCommand(1),

                                new ShooterShotCommand(shootSubsystem, indexSubsystem));

        }

        public static Command holdShot(ShootSubsystem shootSubsystem, Drivetrain drivetrain,
                        VisionSubsystem visionSubsystem, CommandXboxController driverController,
                        ArmAngleSubsystem armAngleSubsystem) {

                return new ParallelCommandGroup(
                                new ShooterAimCommand(visionSubsystem, armAngleSubsystem),
                                new ShootFireCommand(shootSubsystem));

        };

        public static Command centerAndFire(VisionSubsystem visionSubsystem, Drivetrain drivetrain,
                        IndexSubsystem indexSubsystem, ShootSubsystem shootSubsystem,
                        CommandXboxController commandXboxController) {

                return new SequentialCommandGroup(

                                new CenterOnTargetCommand(visionSubsystem, drivetrain,
                                                AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
                                                commandXboxController),
                                new IndexFireCommand(indexSubsystem, shootSubsystem));

        }
}