package frc.robot.commands.elevator.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class QueueL3Action extends Command {
    private final Elevator elevator;
    public QueueL3Action() {
        this.elevator = Elevator.getInstance();
    }

    @Override
    public void initialize() {
        elevator.requestLevel(Elevator.Height.L3);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


    @Override
    public void end(boolean interrupted) {
    }
}
