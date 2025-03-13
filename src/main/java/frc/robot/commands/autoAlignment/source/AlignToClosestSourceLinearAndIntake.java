package frc.robot.commands.autoAlignment.source;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.commands.autoAlignment.LockDriveAxis;
import frc.robot.commands.roller.IntakeCoral;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.roller.Roller;
public class AlignToClosestSourceLinearAndIntake extends ParallelCommandGroup {
    public AlignToClosestSourceLinearAndIntake(XboxController controller) {
        addCommands(
            new WaitUntilCommand( 
                () -> !Roller.getInstance().inRoller()
            ),
            new ParallelCommandGroup(
                new IntakeCoral(),
                // new LinearAlign(
                //     () -> AlignmentUtil.getClosestSourcePose(),
                //     () -> new ChassisSpeeds(),
                //     2
                // )
                new LockDriveAxis(controller, Constants.axinull)
            )
        );
    }
}