package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

public class RunPivotRaw extends Command {
    private final Pivot pivot;
    private final XboxController controller;
    
    public RunPivotRaw(XboxController controller) {
        this.pivot = Pivot.getInstance();
        this.controller = controller;
        
        addRequirements(pivot);
    }

    @Override 
    public void execute() {
        double userIn = MathUtil.applyDeadband(-controller.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND);
        double torque = userIn * 30;

        this.pivot.setTorqueCurrentFOC(torque);

        Logger.recordOutput("RunPivotRaw/torque", torque);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setTorqueCurrentFOC(0);
    }
}
