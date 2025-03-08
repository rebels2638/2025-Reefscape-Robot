package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.climber.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

public class RunClimberRaw extends Command {
    private final Climber climber;
    private final XboxController controller;
    
    public RunClimberRaw(XboxController controller) {
        this.climber = Climber.getInstance();
        this.controller = controller;
        
        addRequirements(climber);
    }

    @Override 
    public void execute() {
        double userIn = MathUtil.applyDeadband(-controller.getLeftY(), Constants.OperatorConstants.LEFT_Y_DEADBAND);
        double torque = userIn * 30;

        this.climber.setTorqueCurrentFOC(torque);

        Logger.recordOutput("RunclimberRaw/torque", torque);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setTorqueCurrentFOC(0);
    }
}
