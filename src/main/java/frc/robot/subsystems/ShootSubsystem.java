package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.ShootLog;
import frc.robot.Model.ShootLogAutoLogged;
import frc.robot.utilities.HoorayConfig;
import frc.robot.utilities.LinearInterpolationTable;
import frc.robot.utilities.SparkFactory;

import java.awt.Point;
import java.awt.geom.Point2D;

public class ShootSubsystem extends SubsystemBase {

    public final CANSparkMax rightMotor;
    public final CANSparkMax leftMotor;
    public final RelativeEncoder rightEncoder;

    private double setpoint = 0;

    private ShootLogAutoLogged shootLogAutoLogged;
    
    private final SimpleMotorFeedforward aimFeed;
    private final BangBangController shooterBangBang;

    private GenericEntry sadf = Shuffleboard.getTab("Asdfsdaf").add("saldjfk", 0).getEntry();
    private GenericEntry vel = Shuffleboard.getTab("Asdfsdaf").add("vel", 0).withWidget(BuiltInWidgets.kGraph).getEntry();

    LinearInterpolationTable shootTable = new LinearInterpolationTable(
        new Point(0, 0)
    );
    

    // 240 inches is the theroetical max shot for the shooter
    public ShootSubsystem() {
        rightMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.shoot1);
        leftMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.shoot2);

        rightEncoder = rightMotor.getEncoder();

        leftMotor.follow(rightMotor, true);

        rightMotor.enableVoltageCompensation(Constants.voltageCompensation);
        leftMotor.enableVoltageCompensation(Constants.voltageCompensation);

        rightMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setIdleMode(IdleMode.kCoast);
        
        rightMotor.burnFlash();
        leftMotor.burnFlash();
        
        aimFeed = new SimpleMotorFeedforward(HoorayConfig.gimmeConfig().getShooterkS(), HoorayConfig.gimmeConfig().getShooterkV());
        shooterBangBang = new BangBangController();

        shootLogAutoLogged = new ShootLogAutoLogged();
    }

    public void changeSetpoint(double set) {
        this.setpoint = set;

    }

    public void stop(){
       setpoint = 0;
       rightMotor.stopMotor();
    }

    private void updateInputs(ShootLog shootLog) {
        shootLog.setpoint = setpoint;
        shootLog.PIDOutput = rightMotor.get();
        Logger.processInputs("Shooter", shootLogAutoLogged);

    }

    @Override
    public void periodic() {
        updateInputs(shootLogAutoLogged);
        

        // setpoint = sadf.getDouble(0);
        vel.setDouble(rightEncoder.getVelocity() / 60);

        if (setpoint == 0) {

            rightMotor.stopMotor();
            leftMotor.stopMotor();
        } else {
            rightMotor.setVoltage(shooterBangBang.calculate(rightEncoder.getVelocity(), setpoint) * 12.0 + (aimFeed.calculate(setpoint / 60) * 0.9));
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
        sysIdRoutineLog.motor("Shoot")
                .voltage(BaseUnits.Voltage.of(rightMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Rotations.of(rightEncoder.getPosition()))
                .angularVelocity(Units.RotationsPerSecond.of(rightEncoder.getVelocity() / 60));
    }

    public double getVelocity() {
        return rightEncoder.getVelocity();
    }
}

