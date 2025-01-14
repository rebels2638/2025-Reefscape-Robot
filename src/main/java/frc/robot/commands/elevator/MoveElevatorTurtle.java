package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorTurtle extends Command {
    private Elevator elevatorSubsystem = Elevator.getInstance();

    public MoveElevatorTurtle() {
    }
    
    @Override
    public void execute() { 
       elevatorSubsystem.setHeightMeters(0.00);
    }
    
    @Override
    public void end(boolean isInterrupted){
        //Add end if need be
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.reachedSetpoint();
    }
}
