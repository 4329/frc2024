package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.ShootLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.HoorayConfig;
import frc.robot.utilities.LinearInterpolationTable;
import frc.robot.utilities.SparkFactory;
import java.awt.geom.Point2D;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShootSubsystem extends SubsystemBase implements LoggedSubsystem {

  public final CANSparkMax rightMotor;
  public final CANSparkMax leftMotor;
  public final RelativeEncoder rightEncoder;
  public final RelativeEncoder leftEncoder;
  public final SparkPIDController rm_aimBot;
  public final SparkPIDController lm_aimBot;
  private GenericEntry rpmSetpointGE;
  private GenericEntry rpmActualGE;
  private GenericEntry rpmActual2GE;
  private LinearInterpolationTable shotTable =
      new LinearInterpolationTable(
          new Point2D.Double(0, 2500),
          new Point2D.Double(1.1, 2500),
          new Point2D.Double(1.2, 2550),
          new Point2D.Double(1.4, 2650),
          new Point2D.Double(1.6, 2700),
          new Point2D.Double(1.8, 2750),
          new Point2D.Double(2, 2800),
          new Point2D.Double(2.2, 2950),
          new Point2D.Double(2.4, 3050),
          new Point2D.Double(2.6, 3150),
          new Point2D.Double(2.8, 3750),
          new Point2D.Double(10, 3800));
  // new Point2D.Double(3, 3300));
  // new Point2D.Double(3.2, 3600),
  // new Point2D.Double(4, 3700));

  private double setpoint = 0;
  private double tolerance = 40; // arbitrary

  private ShootLogAutoLogged shootLogAutoLogged;

  private double lP = 0.00025;
  private double lI = 0.000001;
  private double lD = 0.01;
  private double lFF = 0.000185;
  private double lIZ = 43;

  private double rP = 0.0002;
  private double rI = 0.000001;
  private double rD = 0.1;
  private double rFF = 0.0001811;
  private double rIZ = 43;

  // 240 inches is the theroetical max shot for the shooter
  public ShootSubsystem() {
    rightMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.shoot1);
    leftMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.shoot2);
    rm_aimBot = rightMotor.getPIDController();
    lm_aimBot = leftMotor.getPIDController();
    rightEncoder = rightMotor.getEncoder();
    leftEncoder = leftMotor.getEncoder();

    // leftMotor.follow(rightMotor, true);

    rightMotor.enableVoltageCompensation(12.5);
    leftMotor.enableVoltageCompensation(12.5);

    rightMotor.setIdleMode(IdleMode.kCoast);
    leftMotor.setIdleMode(IdleMode.kCoast);
    leftMotor.setInverted(true);

    rm_aimBot.setP(rP);
    rm_aimBot.setI(rI);
    rm_aimBot.setD(rD);
    rm_aimBot.setFF(rFF);
    rm_aimBot.setIZone(rIZ);

    lm_aimBot.setP(lP);
    lm_aimBot.setI(lI);
    lm_aimBot.setD(lD);
    lm_aimBot.setFF(lFF);
    lm_aimBot.setIZone(lIZ);

    rightMotor.burnFlash();
    leftMotor.burnFlash();

    HoorayConfig.gimmeConfig().getShooterkV();

    shootLogAutoLogged = new ShootLogAutoLogged();

    rpmActualGE = Shuffleboard.getTab("shoot").add("rpm actual 1", 0).getEntry();
    rpmActual2GE = Shuffleboard.getTab("shoot").add("rpm actual 2", 0).getEntry();
    rpmSetpointGE = Shuffleboard.getTab("shoot").add("current rpm setpoint", 0).getEntry();
  }

  public void changeSetpoint(double set) {
    this.setpoint = set;
  }

  public boolean atSetpoint() {

    if (Math.abs(setpoint - getRightVelocity()) <= tolerance
        && Math.abs(setpoint - getLeftVelocity()) <= tolerance) {
      System.out.println("atsetpoint ----");

      return true;
    }
    return false;
  }

  public void shooterDistance(Pose3d pose) {

    setpoint = shotTable.getOutput(Math.sqrt(Math.pow(pose.getZ(), 2) + Math.pow(pose.getX(), 2)));
  }

  public boolean aboveSetpoint() {

    if (leftEncoder.getVelocity() >= (setpoint - tolerance)
        && rightEncoder.getVelocity() >= (setpoint - tolerance)) {
      return true;
    }
    return false;
  }

  public void stop() {
    System.out.println("shootSTOP");

    setpoint = 0;
    rightMotor.stopMotor();
  }

  @Override
  public LoggableInputs log() {
    shootLogAutoLogged.setpoint = setpoint;
    shootLogAutoLogged.rightPIDOutput = rightMotor.getAppliedOutput();
    shootLogAutoLogged.leftPIDOutput = leftMotor.getAppliedOutput();
    shootLogAutoLogged.leftVel = leftEncoder.getVelocity();
    shootLogAutoLogged.rightVel = rightEncoder.getVelocity();
    return shootLogAutoLogged;
  }

  @Override
  public void periodic() {

    // setpoint = sadf.getDouble(0);

    rpmActualGE.setDouble(rightEncoder.getVelocity());
    rpmActual2GE.setDouble(leftEncoder.getVelocity());
    rpmSetpointGE.setDouble(setpoint);

    if (setpoint == 0) {

      rightMotor.stopMotor();
      leftMotor.stopMotor();
    } else {
      rm_aimBot.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
      lm_aimBot.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
    }
  }

  public void setRPM(double rpm) {
    setpoint = rpm;
  }

  public void setVoltage(Measure<Voltage> voltage) {
    rightMotor.setVoltage(voltage.in(BaseUnits.Voltage));
  }

  public void getData(SysIdRoutineLog sysIdRoutineLog) {
    Logger.recordOutput("sdifa", RobotController.getBatteryVoltage());
    Logger.recordOutput("lsdoflsoaolodsflsdfldlsoflso", rightMotor.getAppliedOutput());

    rightMotor.getVoltageCompensationNominalVoltage();
    sysIdRoutineLog
        .motor("Shoot")
        .voltage(
            BaseUnits.Voltage.of(
                rightMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
        .angularPosition(Units.Rotations.of(rightEncoder.getPosition()))
        .angularVelocity(Units.RotationsPerSecond.of(rightEncoder.getVelocity() / 60));
  }

  public double getRightVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getLeftVelocity() {
    return leftEncoder.getVelocity();
  }
}
