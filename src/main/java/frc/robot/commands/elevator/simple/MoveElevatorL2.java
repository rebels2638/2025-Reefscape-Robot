package frc.robot.commands.elevator.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorL2 extends Command {
    private final Elevator elevator;
    public MoveElevatorL2() {
        this.elevator = Elevator.getInstance();
        
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setHeight(0.34);
    }

    @Override
    public boolean isFinished() {
        return elevator.reachedSetpoint();
    }


    @Override
    public void end(boolean interrupted) {
    }
}
