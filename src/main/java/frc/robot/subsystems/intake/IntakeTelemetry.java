package frc.robot.subsystems.intake;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeTelemetry {

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable[] intakes;
    private final BooleanPublisher intakeHasNote;
    private final BooleanPublisher intakeNoteLoaded;
    private final IntegerPublisher risingEdges;
    private final DoublePublisher intakeCurrentDraw;

    public IntakeTelemetry() {
        for (int i = 0; i < 4; i++) {

        }
    }

    private final NetworkTable frontIntakeTable = inst.getTable("Front Intake");
    private final BooleanPublisher frontIntakeHasNote = frontIntakeTable.getBooleanTopic("Front Intake Has Note")
            .publish();
    private final BooleanPublisher frontIntakeNoteLoaded = frontIntakeTable.getBooleanTopic("Front Intake Note Loaded")
            .publish();
    private final IntegerPublisher frontIntakeRisingEdges = frontIntakeTable
            .getIntegerTopic("Front Intake Rising Edges")
            .publish();
    private final IntegerPublisher frontIntakeDraw = frontIntakeTable
            .getIntegerTopic("Front Intake Current Draw")
            .publish();

    private double lastTime = Utils.getCurrentTimeSeconds();

    public void telemeterize(IntakeState state) {
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
    }
}
