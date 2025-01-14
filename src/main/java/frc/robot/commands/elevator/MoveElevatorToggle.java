package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorToggle extends Command {
    private Elevator elevatorSubsystem = Elevator.getInstance();
    private double currHeight;

    public MoveElevatorToggle() {
    }
    
    @Override
    public void initialize() { 
        currHeight = elevatorSubsystem.getShooterHeightMeters();
        if(currHeight < 0.4)
            elevatorSubsystem.setHeightMeters(0.5);
        else
            elevatorSubsystem.setHeightMeters(0);
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

