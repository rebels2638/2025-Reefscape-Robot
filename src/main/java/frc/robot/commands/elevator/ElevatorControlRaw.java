package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.lib.input.XboxController;


public class ElevatorControlRaw extends Command{
    Elevator ElevatorSubsystem = Elevator.getInstance();
    XboxController controller;
    /**
     * This is really for testing, and a raw controller for the operator to use whenever they MUST, not recommend for use when other commands are working
     * @param Elevator
     * @param m_controller
     */
    public ElevatorControlRaw(XboxController m_controller, Elevator m_Elevator){
        controller = m_controller;
        addRequirements(ElevatorSubsystem);
    }

    @Override
    public void execute(){   
        if (controller.getAButton().getAsBoolean()) {
            ElevatorSubsystem.setVoltage(9);
        }

        else if (controller.getBButton().getAsBoolean()) {
            ElevatorSubsystem.setVoltage(0);
        }
    }

    @Override
    //This is a default command, it should never finish theoretically
    public boolean isFinished(){
        return false;
    }

    
}
