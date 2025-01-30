package frc.robot.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.swerve.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.constants.Constants;

/*
This class is NOT A TRADITIONAL SUPERSTRUCTURE CLASS AS SEEN IN OTHER REPOS;
it is meant to serve as link between commands and subsystems when managing game pieces.
It's basic function is to serve as a state machine for game pieces and visualize mechanisms.
This is done to ease simulation support of the above functionality rather than decentralizing
the simulation of inter-dependent aspects of the robot.
 */

// TODO: THIS WILL EVEUALLY BE EXPANED TO FULL MECHANISIM SIMS
public class Superstructure extends SubsystemBase {
    // private final SwerveDrive swerveDrive;
    // private final Elevator elevator;
    // private final Roller roller;
    // private final Pivot pivot;
    private final Claw claw;

    private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();
    private final SuperstructureIO superstructureIO;

    public static enum CoralState {
        OUT_SIDE,
        ROLLER
    }

    public static enum AlgayState {
        OUT_SIDE,
        CLAW
    }

    public Superstructure(
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

        switch (Constants.currentMode) {
            case COMP:
                superstructureIO = new SuperstructureIOProto(
                    // swerveDrive, 
                    // elevator, 
                    // roller, 
                    // pivot, 
                    claw);

                break;

            case PROTO:
                superstructureIO = new SuperstructureIOProto(
                    // swerveDrive, 
                    // elevator, 
                    // roller, 
                    // pivot, 
                    claw);

                break;

            case SIM:
                superstructureIO = new SuperstructureIOProto(
                    // swerveDrive, 
                    // elevator, 
                    // roller, 
                    // pivot, 
                    claw);

                break;

            default:
                superstructureIO = new SuperstructureIO() {};

                break;
        }
    }

    @Override
    public void periodic() {
        superstructureIO.updateInputs(inputs);
        Logger.processInputs("Superstructure", inputs);
    }

    public boolean inClaw() {
        return inputs.algayState == AlgayState.CLAW;
    }

    public boolean inRoller() {
        return inputs.coralState == CoralState.ROLLER;
    }

}
