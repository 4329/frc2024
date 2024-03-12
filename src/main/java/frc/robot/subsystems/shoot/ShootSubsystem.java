package frc.robot.subsystems.shoot;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Model.ShootLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public class ShootSubsystem extends SubsystemBase implements LoggedSubsystem {
   
    ShootIO shootIO;
        private ShootLogAutoLogged shootLogAutoLogged;


    public ShootSubsystem(ShootIO shootIO) {

        shootLogAutoLogged = new ShootLogAutoLogged();
        this.shootIO = shootIO;
    }

    public void changeSetpoint(double set) {
        shootIO.changeSetpoint(set);
    }

    public boolean atSetpoint() {
        return shootIO.atSetpoint();
    }

    public void shooterDistance(Pose3d pose) {
        shootIO.shooterDistance(pose);
    }

    public boolean aboveSetpoint() {
        return shootIO.aboveSetpoint();
    }

    public void stop() {
        shootIO.stop();
    }

    public void setRPM(double rpm) {
        shootIO.setRPM(rpm);
    }

    public void setVoltage(Measure<Voltage> voltage) {
        shootIO.setVoltage(voltage);
    }

    public void getData(SysIdRoutineLog sysIdRoutineLog) {
        shootIO.getData(sysIdRoutineLog);
    }

    @Override
    public void periodic() {
        shootIO.periodic();
    }

    @Override
    public LoggableInputs log() {
        return shootIO.log(shootLogAutoLogged);
    }

}
