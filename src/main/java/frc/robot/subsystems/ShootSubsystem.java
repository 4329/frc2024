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
import frc.robot.utilities.MathUtils;
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
  private GenericEntry rpmSetpoint2GE;
  private GenericEntry rpmActualGE;
  private GenericEntry rpmActual2GE;
  private LinearInterpolationTable shotTable =
      new LinearInterpolationTable(
          new Point2D.Double(0.0, 3300),
          new Point2D.Double(1.7, 3500),
          new Point2D.Double(1.9, 3700),
          new Point2D.Double(2.1, 3750),
          new Point2D.Double(2.3, 3800),
          new Point2D.Double(2.5, 3900),
          new Point2D.Double(2.7, 4050),
          new Point2D.Double(2.9, 4150),
          new Point2D.Double(3.1, 4300),
          new Point2D.Double(3.3, 4450),
          new Point2D.Double(3.5, 4500),
          new Point2D.Double(3.7, 4650),
          new Point2D.Double(3.9, 4750),
          new Point2D.Double(4.1, 4850),
          new Point2D.Double(4.3, 5000),
          new Point2D.Double(4.5, 5050),
          new Point2D.Double(4.7, 5100),
          new Point2D.Double(4.9, 5125),
          new Point2D.Double(5.1, 5150),
          new Point2D.Double(5.35, 5175),
          new Point2D.Double(5.5, 5350),
          new Point2D.Double(10, 5400));
  // new Point2D.Double(3, 3300));
  // new Point2D.Double(3.2, 3600),
  // new Point2D.Double(4, 3700));

  private double setpoint = 0;
  private double tolerance = 40; // arbitrary
  private double speedPercent = 0.90;

  public static final double MAX_SHOT_DISTANCE = 5.7;
  private ShootLogAutoLogged shootLogAutoLogged;

  private double lP = 0.000059;
  private double lI = 0.000001;
  private double lD = 0.1;
  private double lFF = 0.0001736;
  private double lIZ = 30;

  private double rP = 0.00006;
  private double rI = 0.000001;
  private double rD = 0.005;
  private double rFF = 0.0001725;
  private double rIZ = 30;

  // private double rP = 0.0000785;
  // private double rI = 0.000001;
  // private double rD = 0.01;
  // private double rFF = 0.000182;
  // private double rIZ = 43;

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
    rightMotor.setInverted(true);
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

    rpmActualGE = Shuffleboard.getTab("shoot").add("right rpm actual", 0).getEntry();
    rpmActual2GE = Shuffleboard.getTab("shoot").add("left rpm actual", 0).getEntry();
    rpmSetpointGE = Shuffleboard.getTab("shoot").add("current left rpm setpoint", 0).getEntry();
    rpmSetpoint2GE = Shuffleboard.getTab("shoot").add("current right rpm setpoint", 0).getEntry();
  }

  public void changeSetpoint(double set) {
    this.setpoint = set;
  }

  public boolean atSetpoint() {

    if (Math.abs(setpoint * speedPercent - getRightVelocity()) <= tolerance
        && Math.abs(setpoint - getLeftVelocity()) <= tolerance) {
      System.out.println("atsetpoint ----");

      return true;
    }
    return false;
  }

  public void shooterDistance(Pose3d pose) {

    setpoint = shotTable.getOutput(MathUtils.getActualDistanceFromPose(pose));
  }

  public boolean aboveSetpoint() {

    if (leftEncoder.getVelocity() >= (setpoint - tolerance)
        && rightEncoder.getVelocity() >= ((setpoint * speedPercent) - tolerance)) {
      return true;
    }
    return false;
  }

  public void stop() {
    System.out.println("shootSTOP");

    setpoint = 0;
    rightMotor.stopMotor();
    leftMotor.stopMotor();
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
    rpmSetpoint2GE.setDouble(setpoint * speedPercent);

    if (setpoint == 0) {

      rightMotor.stopMotor();
      leftMotor.stopMotor();
    } else {
      rm_aimBot.setReference(setpoint * speedPercent, CANSparkMax.ControlType.kVelocity);
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
