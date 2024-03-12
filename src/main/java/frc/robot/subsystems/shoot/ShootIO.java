package frc.robot.subsystems.shoot;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Model.ShootLogAutoLogged;

public interface ShootIO {

    void changeSetpoint(double set);

    boolean atSetpoint();

    void shooterDistance(Pose3d pose);

    boolean aboveSetpoint();

    void stop();

    void periodic();

    void setRPM(double rpm);

    void setVoltage(Measure<Voltage> voltage);

    void getData(SysIdRoutineLog sysIdRoutineLog);

    double getRightVelocity();

    double getLeftVelocity();

    ShootLogAutoLogged log(ShootLogAutoLogged shootLogAutoLogged);
}