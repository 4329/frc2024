package frc.robot.subsystems.shoot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.NEOConstants;
import frc.robot.Model.ShootLogAutoLogged;

public class ShootIOSim implements ShootIO {

    FlywheelSim flywheelSim;
    DCMotor dcMotor;
    PIDController flywheelPID;

    Mechanism2d mechanism;
    MechanismLigament2d arm;
    private double angle;

    private double setpoint;

    public ShootIOSim() {
        dcMotor = new DCMotor(NEOConstants.nominalVoltage, NEOConstants.stallTorque, NEOConstants.stallCurrent, NEOConstants.freeCurrent, NEOConstants.freeSpeed, 1);
        flywheelSim = new FlywheelSim(dcMotor, 1, 0.000193187908986313);

        flywheelPID = new PIDController(0.1, 0, 0);
        flywheelPID.setSetpoint(0);

        mechanism = new Mechanism2d(1, 1);
        arm = mechanism.getRoot("Flywheel", 0.5, 0.5).append(new MechanismLigament2d("wheel?", 0.5, 0, 10, new Color8Bit(0, 0, 256)));

        Shuffleboard.getTab("Sim").add("Flywheel", mechanism);
    }

    @Override
    public void changeSetpoint(double set) {
        setpoint = set;
    }

    @Override
    public boolean atSetpoint() {
        return flywheelPID.atSetpoint();
    }

    @Override
    public void shooterDistance(Pose3d pose) {
        setpoint = shotTable.getOutput(Math.sqrt((pose.getZ() * pose.getZ()) + (pose.getX() * pose.getX())));
    }

    @Override
    public boolean aboveSetpoint() {
        return flywheelSim.getAngularVelocityRPM() >= setpoint;  
    }

    @Override
    public void stop() {
        setpoint = 0;

        flywheelSim.setInputVoltage(0);
    }

    @Override
    public void setRPM(double rpm) {
        setpoint = rpm;
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {
        flywheelSim.setInputVoltage(voltage.baseUnitMagnitude());
    }

    @Override
    public void getData(SysIdRoutineLog sysIdRoutineLog) {
        sysIdRoutineLog.motor("Shoot")
                .voltage(BaseUnits.Voltage.of(flywheelSim.getOutput(0) * RobotController.getBatteryVoltage()))
                .angularVelocity(Units.RotationsPerSecond.of(flywheelSim.getAngularVelocityRPM() / 60));
    }

    @Override
    public ShootLogAutoLogged log(ShootLogAutoLogged shootLogAutoLogged) {
        return shootLogAutoLogged;
    }

    @Override
    public void periodic() {
        double pidOutput = flywheelPID.calculate(flywheelSim.getAngularVelocityRPM(), setpoint);
        flywheelSim.setInputVoltage(pidOutput);
        flywheelSim.update(0.020);

        angle += (flywheelSim.getAngularVelocityRadPerSec() / (2 * Math.PI)) * 0.020;
        setpoint = 1000;
        System.out.println(flywheelSim.getAngularVelocityRPM() + ", " + pidOutput);
        arm.setAngle(angle);
    }
    
}
