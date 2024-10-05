package frc.robot.Model;

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
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
      addToCache(name);
    }

    List<Trajectory> paths = cachedPaths.get(name);
    drawPath(paths);

    SmartDashboard.putData(field);

    numObjects = paths.size();
    lastName = name;
  }

  private void drawPath(List<Trajectory> paths) {
    // FieldObjects cannot be cleared, so must have their data removed
    // in order to cease being displayed
    for (int i = 0; i < Math.max(paths.size(), numObjects); i++) {
      Trajectory tmp = i < paths.size() ? paths.get(i) : new Trajectory();
      field.getObject(i + "").setTrajectory(tmp);
    }
  }

  private List<Trajectory> mapPaths(List<PathPlannerPath> paths) {
    List<Trajectory> trajectories = new ArrayList<>();
    for (PathPlannerPath p : paths) {
      trajectories.add(pathTrajToTragTraj(p.getTrajectory(new ChassisSpeeds(), new Rotation2d())));
    }
    return trajectories;
  }

  private Trajectory pathTrajToTragTraj(PathPlannerTrajectory pathPlannerTrajectory) {
    return new Trajectory(
        pathPlannerTrajectory.getStates().stream()
            .map(
                (state) ->
                    new Trajectory.State(
                        state.timeSeconds,
                        state.velocityMps,
                        state.accelerationMpsSq,
                        new Pose2d(state.positionMeters, state.targetHolonomicRotation),
                        state.curvatureRadPerMeter))
            .toList());
  }

  private void addToCache(String name) {
    List<Trajectory> toPut;
    try {
      toPut = mapPaths(PathPlannerAuto.getPathGroupFromAutoFile(name));
    } catch (Exception e) {
      toPut = new ArrayList<>();
    }

    cachedPaths.put(name, toPut);
  }
}
