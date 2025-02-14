package frc.robot.commands.elevator.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorL4 extends Command {
    private final Elevator elevator;
    public MoveElevatorL4() {
        this.elevator = Elevator.getInstance();
        
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setHeight(1.38);
    }

    @Override
    public boolean isFinished() {
        return elevator.reachedSetpoint();
    }


    @Override
    public void end(boolean interrupted) {
    }
}
