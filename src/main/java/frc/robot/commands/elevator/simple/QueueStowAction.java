package frc.robot.commands.elevator.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class QueueStowAction extends Command {
    private final Elevator elevator;
    public QueueStowAction() {
        this.elevator = Elevator.getInstance();
        
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.requestLevel(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


    @Override
    public void end(boolean interrupted) {
    }
}
