package frc.robot.literals;

import swerve.CommandSwerveDrivetrain;

public class RobotLiterals {
    private final SwerveDrivetrainLiterals swerveDrivetrainLiterals;
    private final DriveCharacterizationLiterals driveCharacterizationLiterals;

    public RobotLiterals(SwerveDrivetrainLiterals swerveDrivetrainLiterals) {
        this.swerveDrivetrainLiterals = swerveDrivetrainLiterals;
        this.driveCharacterizationLiterals = new DriveCharacterizationLiterals();
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return this.swerveDrivetrainLiterals.getDrivetrain();
    }

    public double getMaxSpeedat12V() {
        return this.swerveDrivetrainLiterals.getMaxSpeedAt12V();
    }

    public double getMaxSpeedAutonomous() {
        return this.swerveDrivetrainLiterals.getMaxSpeedAutonomous();
    }

    public DriveCharacterizationLiterals getDriveCharacterizationLiterals() {
        return driveCharacterizationLiterals;
    }
}
