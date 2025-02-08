package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.elevator.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

public class RunElevatorRaw extends Command {
    private final Elevator elevator;
    private final XboxController controller;
    public RunElevatorRaw(XboxController controller) {
        this.elevator = Elevator.getInstance();
        this.controller = controller;
        
        addRequirements(elevator);
    }

    @Override 
    public void execute() {
        double userIn = MathUtil.applyDeadband(-controller.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND);
        double torque = userIn * 17;

        elevator.setTorqueCurrentFOC(torque);

        Logger.recordOutput("RunElevatorRaw/torque", torque);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setTorqueCurrentFOC(0);
    }
}
