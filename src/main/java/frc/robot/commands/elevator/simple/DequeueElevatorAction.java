package frc.robot.commands.elevator.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class DequeueElevatorAction extends Command {
    private final Elevator elevator;
    public DequeueElevatorAction() {
        this.elevator = Elevator.getInstance();
        
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSetpointSettable(true);
    }

    @Override
    public boolean isFinished() {
        return elevator.reachedSetpoint();
    }


    @Override
    public void end(boolean interrupted) {
        elevator.setSetpointSettable(false);
    }
}
