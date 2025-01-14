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
    public ElevatorControlRaw(XboxController m_controller){
        controller = m_controller;
    }

    @Override
    public void execute(){    
        double height = this.ElevatorSubsystem.getShooterHeightMeters();
        if (controller.getLeftY() > 0.04 || controller.getLeftY() < 0.04) {
            height += controller.getLeftY()*0.54;
        }
        ElevatorSubsystem.setHeightMeters(height);
    }

    @Override
    //This is a default command, it should never finish theoretically
    public boolean isFinished(){
        return false;
    }

    
}
