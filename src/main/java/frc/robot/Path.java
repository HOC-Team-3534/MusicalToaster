package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;

public enum Path {
    DEFAULTPATH("Default Path");

    String pathName;
    PathPlannerPath path;

    Path() {
        this.pathName = this.name();
    }

    Path(String pathName) {
        this.pathName = pathName;
    }

    public void loadPath() {
        path = PathPlannerPath.fromPathFile(pathName);
    }

    public PathPlannerPath getPath() {
        return path;
    }
}