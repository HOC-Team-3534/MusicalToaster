package frc.robot;

public class RobotState {
    int grabNoteIndex = -1;
    boolean activelyGrabbing;
    boolean noteLoaded;
    boolean climbing;
    boolean resetingClimber;
    boolean isExtaking;

    public boolean isActivelyGrabbing() {
        return this.activelyGrabbing;
    }

    public void setActivelyGrabbing(boolean activelyGrabbing) {
        this.activelyGrabbing = activelyGrabbing;
    }

    public boolean isNoteLoaded() {
        return this.noteLoaded;
    }

    public void setNoteLoaded(boolean noteLoaded) {
        this.noteLoaded = noteLoaded;
    }

    public int getGrabNoteIndex() {
        return this.grabNoteIndex;
    }

    public void setGrabNoteIndex(int grabNoteIndex) {
        this.grabNoteIndex = grabNoteIndex;
    }

    public boolean isNoteInRobot() {
        return grabNoteIndex != -1 || noteLoaded;
    }

    public void setClimbing() {
        this.climbing = true;
    }

    public void resetClimbing() {
        this.climbing = false;
    }

    public boolean isClimbing() {
        return this.climbing;
    }

    public void setResetingClimber() {
        this.resetingClimber = true;
    }

    public void resetResetingClimber() {
        this.resetingClimber = false;
    }

    public boolean isResetingClimber() {
        return this.resetingClimber;
    }

    public void setExtaking() {
        this.isExtaking = true;
    }

    public void resetExtaking() {
        this.isExtaking = false;
    }

    public boolean isExtaking() {
        return this.isExtaking;
    }
}
