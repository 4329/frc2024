package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.commands.armCommands.ArmToIntakeCommand;
import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.commands.elevatorCommands.ElevatorCommand;
import frc.robot.commands.indexCommands.IndexCommand;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.commands.indexCommands.IndexSensorCommand;
import frc.robot.commands.indexCommands.OutdexCommand;
import frc.robot.commands.intakeOuttakeCommands.IntakeCommand;
import frc.robot.commands.intakeOuttakeCommands.IntakeSensorCommand;
import frc.robot.commands.intakeOuttakeCommands.OutakeCommand;
import frc.robot.commands.shootCommands.ShootCommand;
import frc.robot.commands.shootCommands.ShooterAimCommand;
import frc.robot.commands.shootCommands.ShooterAimCommandIndefinite;
import frc.robot.commands.shootCommands.ShotRevCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;

public class CommandGroups {

        public static Command intakeFull(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {

                return new ParallelCommandGroup(

                                new IntakeCommand(intakeSubsystem),
                                new IndexCommand(indexSubsystem)

                );

        }

        public static Command intakeWithLineBreakSensor(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem,
                        LineBreakSensorSubsystem lineBreakSensorSubsystem, ArmAngleSubsystem armAngleSubsystem) {

                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new ArmCommand(armAngleSubsystem, ArmAngle.INTAKE),
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

        public static Command aim(Drivetrain m_robotDrive, VisionSubsystem visionSubsystem,
                        CommandXboxController driverController, ArmAngleSubsystem armAngleSubsystem) {

                return new ParallelCommandGroup(
                                new CenterOnTargetCommand(visionSubsystem, m_robotDrive,
                                                AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(), driverController),
                                new ShooterAimCommand(visionSubsystem, armAngleSubsystem));

        }

        public static Command holdShot(ShootSubsystem shootSubsystem, Drivetrain drivetrain,
                        VisionSubsystem visionSubsystem, CommandXboxController driverController,
                        ArmAngleSubsystem armAngleSubsystem) {

                return new ParallelCommandGroup(
                                new ShooterAimCommand(visionSubsystem, armAngleSubsystem),
                                new ShootFireCommand(shootSubsystem));

        };

        public static Command elevatorAndAngleToAmp(ShootSubsystem shootSubsystem, IndexSubsystem indexSubsystem,
                        ArmAngleSubsystem armAngleSubsystem, ElevatorSubsystem elevatorSubsystem) {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new ElevatorToAmpCommand(elevatorSubsystem),
                                                new ArmCommand(armAngleSubsystem, ArmAngle.ARMAMP)));
        }

        public static Command FullZeroCommand(ElevatorSubsystem elevatorSubsystem,
                        ArmAngleSubsystem armAngleSubsystem) {

                return new SequentialCommandGroup(

                                new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.ZERO),

                                new ArmCommand(armAngleSubsystem, ArmAngle.ZERO));

        }

        public static Command centerAndFire(VisionSubsystem visionSubsystem, Drivetrain drivetrain,
                        IndexSubsystem indexSubsystem, ShootSubsystem shootSubsystem,
                        CommandXboxController commandXboxController) {

                return new SequentialCommandGroup(

                                new CenterOnTargetCommand(visionSubsystem, drivetrain,
                                                AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
                                                commandXboxController),
                                new IndexFireCommand(indexSubsystem, shootSubsystem));

        }


        public static Command shoot(ShootSubsystem shootSubsystem, IndexSubsystem indexSubsystem, VisionSubsystem visionSubsystem, Drivetrain drivetrain, CommandXboxController commandXboxController, ArmAngleSubsystem armAngleSubsystem) {
                                
                
                System.out.println("shoot1");

                return new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new ShotRevCommand(shootSubsystem),
                              //  new CenterOnTargetCommandIndefinite(visionSubsystem, drivetrain, AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(), commandXboxController),
                                new ShooterAimCommandIndefinite(visionSubsystem, armAngleSubsystem)

                        ),

                        new ParallelRaceGroup(

                                new ShooterShotCommand(shootSubsystem, indexSubsystem),
                             //   new CenterOnTargetCommandIndefinite(visionSubsystem, drivetrain, AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(), commandXboxController),
                                new ShooterAimCommandIndefinite(visionSubsystem, armAngleSubsystem)


                        ),
                        
                        new ArmCommand(armAngleSubsystem, ArmAngle.INTAKE)
                        );




        }

}