// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.concurrent.Callable;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EnabledDebugModes;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.FieldCentricWithProperDeadband;
import frc.robot.utils.ControllerUtils;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	// The driver station connected controllers are defined here...
	private static double MaxAngularRate = 4 * Math.PI;

	private static final CommandXboxController driverController = new CommandXboxController(0);
	// private static final CommandXboxController operatorController = new
	// CommandXboxController(1);
	static SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterRotation = new SlewRateLimiter(2.5);
	private final static CommandSwerveDrivetrain drivetrain = Constants.ROBOT.getDrivetrain();
	private final static FieldCentricWithProperDeadband drive = new FieldCentricWithProperDeadband()
			.withDeadband(Constants.ROBOT.getMaxSpeedat12V() * 0.15).withRotationalDeadband(MaxAngularRate * 0.15)
			.withMaxSpeed(Constants.ROBOT.getMaxSpeedat12V()).withMaxAngularSpeed(MaxAngularRate)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public static void initialize() {
		// Configure the trigger bindings
		configureBindings();

		// Set Default Commands for Subsystems
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(
						() -> drive
								.withVelocityX(
										// Drive forward with negative Y (forward)
										-driverController.getLeftY() * Constants.ROBOT.getMaxSpeedat12V()
												* getCoordinateSystemInversionDriving())
								.withVelocityY(
										// Drive left with negative X (left)
										-driverController.getLeftX() * Constants.ROBOT.getMaxSpeedat12V()
												* getCoordinateSystemInversionDriving())
								.withRotationalRate(
										// Drive counterclockwise with negative X (left)
										-driverController.getRightX() * MaxAngularRate)
								.withCreepEnabled(driverController.getHID().getLeftBumper())));

		// var idle = new TurretRequest.Idle();

		// turret.setDefaultCommand(turret.applyRequest(() -> idle, () -> shooterOff));

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
	 * with an arbitrary predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s
	 * subclasses for {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private static void configureBindings() {
		TGR.Characterize.tgr().whileTrue(drivetrain.characterizeDrive(1.0, 4.0));

		// reset the field-centric heading on left bumper press
		TGR.ResetFieldRelative.tgr().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
	}

	public static int getCoordinateSystemInversionDriving() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() && alliance.get().equals(Alliance.Red) ? -1 : 1;
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public static Command getAutonomousCommand() {
		return Commands.none();
	}

	public enum TGR {
		ResetFieldRelative(driverController.start()),

		// Below are debugging actions

		Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled));

		Trigger trigger;

		TGR(Trigger trigger) {
			this.trigger = trigger;
		}

		public Trigger tgr() {
			return trigger.and(() -> !Robot.isAutonomous);
		}
	}

	public enum AXS {
		Drive_ForwardBackward(
				() -> slewRateLimiterX.calculate(-ControllerUtils.modifyAxis(driverController.getLeftY()))),
		Drive_LeftRight(() -> slewRateLimiterY.calculate(-ControllerUtils.modifyAxis(driverController.getLeftX()))),
		Drive_Rotation(
				() -> slewRateLimiterRotation.calculate(-ControllerUtils.modifyAxis(driverController.getRightX())));

		Callable<Double> callable;

		AXS(Callable<Double> callable) {
			this.callable = callable;
		}

		public double getAxis() {
			try {
				return callable.call().doubleValue();
			} catch (Exception ex) {
				return 0.0;
			}
		}
	}
}