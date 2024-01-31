package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    Supplier<Rotation2d> robotHeading;

    public Turret(Supplier<Rotation2d> robotHeading) {
        super();

        this.robotHeading = robotHeading;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public Rotation2d getRobotRelativeAngle() {
        return null;
    }

    public Rotation2d getFieldRelativeAngle() {
        return getRobotRelativeAngle().plus(robotHeading.get());
    }

}
