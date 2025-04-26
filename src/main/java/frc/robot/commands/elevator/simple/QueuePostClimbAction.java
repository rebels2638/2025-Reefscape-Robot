package frc.robot.commands.elevator.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class QueuePostClimbAction extends Command {
    private final Elevator elevator;
    public QueuePostClimbAction() {
        this.elevator = Elevator.getInstance();
    }

    @Override
    public void initialize() {
        elevator.requestLevel(Elevator.Height.POST_CLIMB);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


    @Override
    public void end(boolean interrupted) {

    }
}
