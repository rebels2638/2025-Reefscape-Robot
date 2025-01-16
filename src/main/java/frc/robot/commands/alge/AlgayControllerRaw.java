package frc.robot.commands.alge;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.Roller;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.RebelUtil;


public class AlgayControllerRaw extends Command{
    private final Roller roller;
    private final XboxController controller;
    /**
     * This is really for testing, and a raw controller for the operator to use whenever they MUST, not recommend for use when other commands are working
     * @param Elevator
     * @param m_controller
     */
    public AlgayControllerRaw(XboxController m_controller, Roller roller){
        controller = m_controller;
        this.roller = roller;

        addRequirements(roller);
    }

    @Override
    public void execute(){   
        roller.setVoltage(8 * RebelUtil.linearDeadband(-controller.getLeftY(), 0.15));
    }

    @Override
    //This is a default command, it should never finish theoretically
    public boolean isFinished(){
        return false;
    }

    
}
