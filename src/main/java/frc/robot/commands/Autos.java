package frc.robot.commands;

import org.ejml.equation.Sequence;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL4Action;
import frc.robot.constants.Constants;

public class Autos {
    public static final Command start_right_2xL4 = 
        new SequentialCommandGroup(
            loadResetPoseCommand("fromProcessorStartToTopRightLB"),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new QueueL4Action(),
                    new isElevatorExtendable(),
                    new DequeueElevatorAction()
                ),
                loadPathFollowCommand("fromProcessorStartToTopRightLB")
            )
            
        );

    public static final Command test1 = 
        new SequentialCommandGroup(
            loadResetPoseCommand("Test1"),
            loadPathFollowCommand("Test1")
        );

    public static final PathPlannerPath loadPath(String name) {
        try {
            // Load the path you want to follow using its name in the GUI
            return PathPlannerPath.fromChoreoTrajectory(name);

        } catch (Exception e) {
            DriverStation.reportError("Oh shit, ur fucked haha " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    public static final Command loadPathFollowCommand(String name) {
        return AutoBuilder.followPath(loadPath(name));
    }

    public static final Command loadResetPoseCommand(String name) {
        PathPlannerPath path = loadPath(name);
        return
            Constants.shouldFlipPath() ?
            new InstantCommand(  // flip starting pose
                () -> RobotState.getInstance().resetPose(
                    FlippingUtil.flipFieldPose(
                        path.getStartingHolonomicPose().get()
                    )
                )
            ) :
            new InstantCommand(  // flip starting pose
                () -> RobotState.getInstance().resetPose(
                    path.getStartingHolonomicPose().get()
                )
            );
    }

    private Autos() {}
}
