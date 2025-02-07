package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorStow extends Command {
    private final Elevator elevator;
    public MoveElevatorStow() {
        this.elevator = Elevator.getInstance();
        
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setHeight(.5);
    }

    @Override
    public boolean isFinished() {
        return elevator.reachedSetpoint();
    }


    @Override
    public void end(boolean interrupted) {
    }
}
