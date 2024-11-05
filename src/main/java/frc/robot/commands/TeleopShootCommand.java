package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.commands.driveCommands.CenterOnTargetCommandIndefinite;
import frc.robot.commands.elevatorCommands.ElevatorCommand;
import frc.robot.commands.elevatorCommands.ElevatorShootCommand;
import frc.robot.commands.elevatorCommands.ElevatorShootCommandIndefinite;
import frc.robot.commands.shootCommands.ShooterAimCommand;
import frc.robot.commands.shootCommands.ShooterAimCommandIndefinite;
import frc.robot.commands.shootCommands.ShooterShotCommand;
import frc.robot.commands.shootCommands.ShotRevCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LightSubsystem.LEDPattern;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;
import frc.robot.utilities.UnInstantCommand;

public class TeleopShootCommand extends SequentialCommandGroup {
  public TeleopShootCommand(
      ShootSubsystem shootSubsystem,
      IndexSubsystem indexSubsystem,
      Drivetrain drivetrain,
      VisionSubsystem visionSubsystem,
      CommandXboxController commandXboxController,
      ElevatorSubsystem elevatorSubsystem,
      ArmAngleSubsystem armAngleSubsystem,
      LightSubsystem lightSubsystem) {

    super(
        new UnInstantCommand(() -> lightSubsystem.setLEDPattern(LEDPattern.BLUE)),
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                new ShooterAimCommand(visionSubsystem, armAngleSubsystem, elevatorSubsystem),
                new ElevatorShootCommand(elevatorSubsystem, visionSubsystem)),
            new ShotRevCommand(shootSubsystem, visionSubsystem).withTimeout(3),
            new CenterOnTargetCommandIndefinite(
                visionSubsystem,
                drivetrain,
                AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
                commandXboxController)),
        new ParallelRaceGroup(
            new ShooterShotCommand(shootSubsystem, indexSubsystem, visionSubsystem).withTimeout(5),
            new CenterOnTargetCommandIndefinite(
                visionSubsystem,
                drivetrain,
                AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
                commandXboxController),
            new ShooterAimCommandIndefinite(visionSubsystem, armAngleSubsystem, elevatorSubsystem),
            new ElevatorShootCommandIndefinite(elevatorSubsystem, visionSubsystem)),
        new ParallelCommandGroup(
            new ArmCommand(armAngleSubsystem, ArmAngle.INTAKE),
            new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.ZERO)),
        new UnInstantCommand(() -> lightSubsystem.setLEDPattern(LEDPattern.NOTHING)));
  }
}
