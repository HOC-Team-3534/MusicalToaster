// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.concurrent.Callable;

import org.opencv.features2d.FlannBasedMatcher;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EnabledDebugModes;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRequest.ControlIntake;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.turret.ShooterRequest;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretRequest;
import frc.robot.subsystems.turret.ShooterRequest.ControlShooter;
import frc.robot.subsystems.turret.TurretRequest.AimForAmp;
import frc.robot.subsystems.turret.TurretRequest.AimForSpeaker;
import frc.robot.subsystems.turret.TurretRequest.IndexFromIntake;
import swerve.CommandSwerveDrivetrain;

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
	private double MaxAngularRate = 4 * Math.PI;

	private static final CommandXboxController driverController = new CommandXboxController(0);
	private static final CommandXboxController operatorController = new CommandXboxController(1);
	static SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(2.5);
	static SlewRateLimiter slewRateLimiterRotation = new SlewRateLimiter(2.5);
	private static final SendableChooser<Callable<Command>> autonChooser = new SendableChooser<>();
	private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
	private final FieldCentricWithProperDeadband drive = new FieldCentricWithProperDeadband()
			.withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.15).withRotationalDeadband(MaxAngularRate * 0.15)
			.withMaxSpeed(TunerConstants.kSpeedAt12VoltsMps).withMaxAngularSpeed(MaxAngularRate)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

	private final Intake intake = new Intake();
	private final ControlIntake runIntake = new ControlIntake().withIntakePercent(0.5);
	private final ControlIntake runExtake = new ControlIntake().withIntakePercent(0.5).withIntakeReversed(true);
	private final ControlIntake stopIntake = new ControlIntake().withIntakePercent(0.0);
	private final ControlShooter shooterOff = new ControlShooter().withVoltage(0);
	private final ControlShooter shooterAmp = new ControlShooter().withVoltage(3);// TODO Find these valuess
	private final ControlShooter shooterSpeaker = new ControlShooter().withVoltage(6);// TODO Find these

	private final Turret turret = new Turret(() -> drivetrain.getState(), () -> drivetrain.getChassisSpeeds());
	private final IndexFromIntake indexFromIntake = new IndexFromIntake().withRollerOutput(0.25)
			.withRotateTolerance(Rotation2d.fromDegrees(1)).withTilt(Rotation2d.fromDegrees(-35))
			.withIntakeState(() -> intake.getState());// TODO
	// Tune
	// all,
	// values
	private final AimForSpeaker aimForSpeaker = new AimForSpeaker();
	private final AimForAmp aimForAmp = new AimForAmp();

	private final boolean CHARACTERIZATION_ENABLED = true;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		// Set Default Commands for Subsystems

		// Autonomous Command Sendable Chooser
		autonChooser.setDefaultOption("No Auton", () -> Commands.none());

		// Show Status of Subsystems on Dashboard
		SmartDashboard.putData(autonChooser);
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
	private void configureBindings() {
		// The following triggered commands are for debug purposes only
		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(
						() -> drive.withVelocityX(-driverController.getLeftY() * TunerConstants.kSpeedAt12VoltsMps) // Drive
								// forward
								// with
								// negative Y (forward)
								.withVelocityY(-driverController.getLeftX() * TunerConstants.kSpeedAt12VoltsMps) // Drive
																													// left
								// with
								// negative
								// X
								// (left)
								.withRotationalRate(-driverController.getRightX() * MaxAngularRate)
								.withCreepEnabled(driverController.getRightTriggerAxis() > 0.15) // Drive
																									// counterclockwise
																									// with
				// negative X (left)
				));

		intake.setDefaultCommand(intake.applyRequest(() -> stopIntake));

		turret.setDefaultCommand(
				turret.applyRequest(() -> indexFromIntake, () -> shooterOff));

		driverController.a().and(() -> CHARACTERIZATION_ENABLED).whileTrue(drivetrain.characterizeDrive(1.0, 4.0));
		driverController.b().whileTrue(drivetrain
				.applyRequest(
						() -> point.withModuleDirection(
								new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

		// reset the field-centric heading on left bumper press
		TGR.ResetFieldRelative.tgr().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

		TGR.Intake.tgr().whileTrue(intake.applyRequest(() -> runIntake));

		TGR.ShootSpeaker.tgr().whileTrue(
				turret.applyRequest(() -> indexFromIntake, () -> shooterSpeaker)
						.until(() -> turret.getState().noteLoaded)
						.andThen(turret.applyRequest(() -> aimForSpeaker, () -> shooterSpeaker)));
		TGR.ShootAmp.tgr().whileTrue(
				turret.applyRequest(() -> indexFromIntake, () -> shooterAmp)
						.until(() -> turret.getState().noteLoaded)
						.andThen(turret.applyRequest(() -> aimForAmp, () -> shooterAmp)));

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}
		drivetrain.registerTelemetry(logger::telemeterize);

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		try {
			return autonChooser.getSelected().call();
		} catch (Exception e) {
			e.printStackTrace();
		}
		return Commands.none();
	}

	public enum TGR {
		Creep(driverController.leftBumper()),
		Characterize(driverController.a().and(() -> EnabledDebugModes.CharacterizeEnabled)),
		ResetFieldRelative(driverController.start()),
		Intake(driverController.rightTrigger(0.15)),
		Extake(driverController.rightBumper()),
		ShootSpeaker(driverController.x()),
		ShootAmp(driverController.b());

		Trigger trigger;

		TGR(Trigger trigger) {
			this.trigger = trigger;
		}

		public Trigger tgr() {
			return trigger.and(() -> !Robot.isAutonomous);
		}

		public boolean bool() {
			return trigger.getAsBoolean();
		}
	}

	public enum AXS {
		Drive_ForwardBackward(() -> slewRateLimiterX.calculate(-modifyAxis(driverController.getLeftY()))),
		Drive_LeftRight(() -> slewRateLimiterY.calculate(-modifyAxis(driverController.getLeftX()))),
		Drive_Rotation(() -> slewRateLimiterRotation.calculate(-modifyAxis(driverController.getRightX())));

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

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0;
		}
	}

	private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.1);
		// Square the axis
		value = Math.copySign(value * value, value);
		return value;
	}
}
