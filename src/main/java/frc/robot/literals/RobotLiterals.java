package frc.robot.literals;

import java.util.Optional;

import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.path.IPathPlanner;

public class RobotLiterals {
    private final SwerveDrivetrainLiterals swerveDrivetrainLiterals;
    private final DriveCharacterizationLiterals driveCharacterizationLiterals;
    private final CameraLiterals cameraLiterals;
    private final TurretTalonConfigLiterals turretTalonConfigLiterals;

    public RobotLiterals(
            SwerveDrivetrainLiterals swerveDrivetrainLiterals,
            CameraLiterals cameraLiterals,
            TurretTalonConfigLiterals turretTalonConfigLiterals) {
        this.swerveDrivetrainLiterals = swerveDrivetrainLiterals;
        this.cameraLiterals = cameraLiterals;
        this.turretTalonConfigLiterals = turretTalonConfigLiterals;

        this.driveCharacterizationLiterals = new DriveCharacterizationLiterals();
    }

    public Optional<CommandSwerveDrivetrain> getDrivetrain() {
        return this.swerveDrivetrainLiterals.getDrivetrain();
    }

    public double getMaxSpeedat12V() {
        return this.swerveDrivetrainLiterals.getMaxSpeedAt12V();
    }

    public double getMaxSpeedAutonomous() {
        return this.swerveDrivetrainLiterals.getMaxSpeedAutonomous();
    }

    public IPathPlanner getPathPlanner() {
        return this.swerveDrivetrainLiterals.getPathPlanner();
    }

    public CameraLiterals getCameraLiterals() {
        return this.cameraLiterals;
    }

    public TurretTalonConfigLiterals getTurretTalonConfigLiterals() {
        return this.turretTalonConfigLiterals;
    }

    public DriveCharacterizationLiterals getDriveCharacterizationLiterals() {
        return driveCharacterizationLiterals;
    }
}
