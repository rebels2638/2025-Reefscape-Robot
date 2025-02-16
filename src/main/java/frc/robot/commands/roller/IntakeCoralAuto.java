package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.roller.Roller;

public class IntakeCoralAuto extends ConditionalCommand {
    public IntakeCoralAuto() {
        super(
            new IntakeCoral(), 
            new InstantCommand(),
            () -> 
                RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(
                    AlignmentUtil.getClosestSourcePose().getTranslation()) <= 2 && 
                !Roller.getInstance().inRoller()
        );
    }
}
