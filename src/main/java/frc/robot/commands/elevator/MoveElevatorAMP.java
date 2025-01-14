package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorAMP extends Command {
    private Elevator elevatorSubsystem = Elevator.getInstance();

    public MoveElevatorAMP() {
    }
    
    @Override
    public void initialize() { 
       elevatorSubsystem.setHeightMeters(0.50); // after one increment of "y"
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
