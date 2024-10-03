package frc.robot.Model;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDisp extends SendableChooser<Command> {

    private final Map<Command, String> autoNames;
    private final Map<String, List<Trajectory>> cachedPaths;
    private Field2d field;
    private String lastName;
    private int numObjects;

    public AutoDisp() {
        autoNames = new HashMap<>();
        cachedPaths = new HashMap<>();
        field = new Field2d();
    }

    @Override
    public void addOption(String key, Command value) {
        autoNames.put(value, key);
        super.addOption(key, value);
    }

    public String getAutoName(Command command) {
        return autoNames.getOrDefault(command, "");
    }

    public void drawPath() {
        String name = getAutoName(getSelected());
        if (name == lastName || name == "") return;

        if (!cachedPaths.containsKey(name)) {
            cachedPaths.put(name, mapPaths(PathPlannerAuto.getPathGroupFromAutoFile(name)));
        }

        List<Trajectory> paths = cachedPaths.get(name);
        for (int i = 0; i < Math.max(paths.size(), numObjects); i++) {
            Trajectory tmp = i < paths.size() ? paths.get(i) : new Trajectory();
            field.getObject(i + "").setTrajectory(tmp);
        }
        numObjects = paths.size();
        SmartDashboard.putData(field);
        lastName = name;
    }

    private List<Trajectory> mapPaths(List<PathPlannerPath> paths) {
        List<Trajectory> trajectories = new ArrayList<>();
        for (int i = 0; i < paths.size(); i++) {
            trajectories.add(
                    pathTrajToTragTraj(
                            paths.get(i).getTrajectory(new ChassisSpeeds(), new Rotation2d())));
        }
        return trajectories;
    }

    private Trajectory pathTrajToTragTraj(PathPlannerTrajectory pathPlannerTrajectory) {
        return new Trajectory(
                pathPlannerTrajectory.getStates().stream()
                        .map(
                                (state) -> new Trajectory.State(
                                        state.timeSeconds,
                                        state.velocityMps,
                                        state.accelerationMpsSq,
                                        new Pose2d(state.positionMeters, state.targetHolonomicRotation),
                                        state.curvatureRadPerMeter))
                        .toList());
    }
}
