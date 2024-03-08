package frc.robot.utils.publishing;

import java.util.LinkedList;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;

public class PathPublisher {

    String selectedAuto = "";
    double selectedMaxAutoPaths;
    LinkedList<PathPlannerPath> autoPaths;
    int pathIndexOnField;
    double totalTimeCurrentTrajectory;
    Timer trajectoryOnFieldTimer = new Timer();

}
