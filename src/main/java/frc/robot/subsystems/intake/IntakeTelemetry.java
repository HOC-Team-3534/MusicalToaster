package frc.robot.subsystems.intake;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.subsystems.intake.Intake.IntakeDirection;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeTelemetry {

        private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
        private final IntakeTable[] intakes = new IntakeTable[4];

        class IntakeTable {
                final NetworkTable table;
                final BooleanPublisher seeingNote, noteInPosition;
                final StringPublisher intakeDirection;
                final String name;

                public IntakeTable(int side) {
                        this.name = "Intake " + side;
                        this.table = inst.getTable(this.name);
                        this.seeingNote = this.table.getBooleanTopic("Seeing Note").publish();
                        this.noteInPosition = this.table.getBooleanTopic("Note In Position").publish();
                        this.intakeDirection = this.table.getStringTopic("Intake Direction").publish();
                }

                public void telemeterize(boolean seeingNote, boolean noteInPosition, IntakeDirection intakeDirection) {
                        this.seeingNote.set(seeingNote);
                        this.noteInPosition.set(noteInPosition);
                        this.intakeDirection.set(intakeDirection.name());
                }
        }

        public IntakeTelemetry() {
                for (int i = 0; i < intakes.length; i++) {
                        intakes[i] = new IntakeTable(i);
                }
        }

        public void telemeterize(IntakeState state) {
                for (int i = 0; i < intakes.length; i++) {
                        intakes[i].telemeterize(state.seeingNote[i], state.noteInPosition[i],
                                        state.intakeDirection[i]);
                }
        }
}
