package frc.robot.commands.shootCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.utilities.ShotRpms;

public class PassingShotCommand extends Command {

  private ShootSubsystem shootSubsystem;
  private IndexSubsystem indexSubsystem;
  private Timer timer = new Timer();
  private boolean shot = false;

  public PassingShotCommand(
      ShootSubsystem shootSubsystem,
      // ArmAngleSubsystem armAngleSubsystem,
      IndexSubsystem indexSubsystem) {
    this.shootSubsystem = shootSubsystem;
    // this.armAngleSubsystem = armAngleSubsystem;
    this.indexSubsystem = indexSubsystem;
    addRequirements(shootSubsystem, indexSubsystem);
  }

  @Override
  public void initialize() {
    shot = false;
    timer.reset();
    // armAngleSubsystem.setArmAngle(ArmAngle.PASS);
    shootSubsystem.setRPM(ShotRpms.PASS.getValue());
  }

  @Override
  public void execute() {
    if (shootSubsystem.aboveSetpoint() && !shot) {
      timer.start();
      indexSubsystem.inShoot();
      shot = true;
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.6);
  }

  @Override
  public void end(boolean interrupted) {
    shootSubsystem.stop();
    indexSubsystem.stop();
    // armAngleSubsystem.setArmAngle(ArmAngle.INTAKE);
    timer.stop();
  }
}
