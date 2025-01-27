package frc.robot.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.constants.claw.ClawConfigProto;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.swerve.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.superstructure.Superstructure.AlgayState;
import frc.robot.superstructure.Superstructure.CoralState;

public class SuperstructureIOProto implements SuperstructureIO {
    private final ClawConfigProto config = ClawConfigProto.getInstance();

    // private final SwerveDrive swerveDrive;
    // private final Elevator elevator;
    // private final Roller roller;
    // private final Pivot pivot;
    private final Claw claw;

    private final LinearFilter currentFilter =
        LinearFilter.highPass(config.getHighPassFilterTimeConstant(), 0.02);

    public SuperstructureIOProto(
        // SwerveDrive swerveDrive,
        // Elevator elevator, 
        // Roller roller, 
        // Pivot pivot, 
        Claw claw) {
        // this.swerveDrive = swerveDrive;
        // this.elevator = elevator; 
        // this.roller = roller;
        // this.pivot = pivot;
        this.claw = claw;
    }

    @Override
    public void updateInputs(SuperstructureIOInputs inputs) {
        double filteredSpeed = currentFilter.calculate(claw.getVelocityRadPerSec());
        if (filteredSpeed >= config.getHighPassFilterUpperTrip()) {
            inputs.algayState = AlgayState.CLAW;
        }
        else if (filteredSpeed <= config.getHighPassFilterLowerTrip()) {
            inputs.algayState = AlgayState.CLAW;
        }
        Logger.recordOutput("Superstructure/Claw/filteredSpeed", filteredSpeed);
    }
}