package frc.robot.utils;

import java.util.LinkedList;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;

public class PathUtils {

    public static LinkedList<PathPlannerPath> getAutoPaths(String autoName, int maxAutopaths) {
        var paths = new LinkedList<PathPlannerPath>();
        var pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        for (int i = 0; i < maxAutopaths && i < pathGroup.size(); i++) {
            paths.add(pathGroup.get(i));
        }
        return paths;
    }

    public static Pose2d getAutoStartingPose(String autoName) {
        return PathPlannerAuto.getStaringPoseFromAutoFile(autoName);
    }

}
