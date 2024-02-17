package frc.robot.utils.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GoalFinalEquation {
	Translation2d robotPosition, goalPosition, robotVelocity;

	public GoalFinalEquation withRobotPosition(Translation2d robotPosition) {
		this.robotPosition = robotPosition;
		return this;
	}

	public GoalFinalEquation withGoalPosition(Translation2d goalPosition) {
		this.goalPosition = goalPosition;
		return this;
	}

	public GoalFinalEquation withRobotVelocity(ChassisSpeeds robotVelocity) {
		this.robotVelocity = new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
		return this;
	}

	public Translation2d getRobotToGoal() {
		return goalPosition.minus(robotPosition);
	}

	public Translation2d getGoalFinal(double timeOfFlight) {
		return goalPosition.minus(robotPosition).minus(robotVelocity.times(timeOfFlight));
	}

}
