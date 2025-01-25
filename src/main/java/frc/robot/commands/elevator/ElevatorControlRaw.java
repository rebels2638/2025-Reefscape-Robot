// package frc.robot.commands.elevator;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.lib.input.XboxController;
// import frc.robot.lib.util.RebelUtil;


// public class ElevatorControlRaw extends Command{
//     private final Elevator elevator;
//     private final XboxController controller;
//     /**
//      * This is really for testing, and a raw controller for the operator to use whenever they MUST, not recommend for use when other commands are working
//      * @param Elevator
//      * @param m_controller
//      */
//     public ElevatorControlRaw(XboxController m_controller, Elevator elevator){
//         controller = m_controller;
//         this.elevator = elevator;
//         addRequirements(elevator);
//     }

//     @Override
//     public void execute(){   
//         elevator.setVoltage(5 * RebelUtil.linearDeadband(-controller.getLeftY(), 0.15));
//     }

//     @Override
//     //This is a default command, it should never finish theoretically
//     public boolean isFinished(){
//         return false;
//     }

    
// }
