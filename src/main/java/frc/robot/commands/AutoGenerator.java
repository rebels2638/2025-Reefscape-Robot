package frc.robot.commands;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.autoAlignment.reef.AlignToLeftBranchLinear;
import frc.robot.commands.autoAlignment.reef.AlignToRightBranchLinear;
import frc.robot.commands.autoAlignment.reef.WarmUpElevatorReef;
import frc.robot.commands.complex.superstructure.ScoreL1Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL2Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL3Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL4Superstructure;
import frc.robot.commands.elevator.simple.QueueL1Action;
import frc.robot.commands.elevator.simple.QueueL2Action;
import frc.robot.commands.elevator.simple.QueueL3Action;
import frc.robot.commands.elevator.simple.QueueL4Action;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.constants.Constants;

import com.pathplanner.lib.util.FlippingUtil;

public class AutoGenerator {
    private AutoGenerator() {}

    /*
     * Generates a auto routine based of the string of parameters
     * 
     * @ param params made up of path names and what branch to score on:
     *  - "path" -> follows path. Must be the first param. Needs to be a real path in choreo - ex. "processorStartToTopRightRB"
     *  - "branch / level " -> where to score - ex. "RB L4" -> scores on L4 on the closest right branch (driver station relative)
     *  - "intake" -> will schedule a intake command. Pass after the drive to source command
     *    this could be passed before a source going path ex. -> "intake", "fromRightSourceTopToBottomRightRB"
     */
    public static Command generateAuto(String... params) {
        SequentialCommandGroup auto = new SequentialCommandGroup();
        ArrayList<Command> commands = new ArrayList<>();

        try {
            PathPlannerPath firstPath = PathPlannerPath.fromChoreoTrajectory(params[0]);
            auto.addCommands(
                Constants.shouldFlipPath() ?
                new InstantCommand(  // flip starting pose
                    () -> RobotState.getInstance().resetPose(
                        FlippingUtil.flipFieldPose(
                            firstPath.getStartingHolonomicPose().get()
                        )
                    )
                ) :
                new InstantCommand(  // flip starting pose
                    () -> RobotState.getInstance().resetPose(
                        firstPath.getStartingHolonomicPose().get()
                    )
                )
            );
        } 
        catch (Exception e) {
            DriverStation.reportError("Oh shit, ur fucked haha " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
        
        for (int i = 0; i < params.length; i++) {
            String param = params[i];

            if (param.split(" ").length == 2) {
                Command alignCommand;
                Command scoreCommand;
                Command raiseElevatorCommand1;
                Command raiseElevatorCommand2;

                try {
                    String[] split = param.split(" ");
                    switch (split[0]) { // auto align because we need maximal accuracy
                        case "RB":
                            alignCommand = new AlignToRightBranchLinear();
                            break;
                    
                        case "LB":
                            alignCommand = new AlignToLeftBranchLinear();
                            break;
    
                        default:
                            throw new IOException("auto generator can not parse branch");
                    }
    
                    switch (split[1]) {
                        case "L1":
                            scoreCommand = new ScoreL1Superstructure();
                            raiseElevatorCommand1 = new QueueL1Action();
                            raiseElevatorCommand2 = new QueueL1Action();

                            break;
                    
                        case "L2":
                            scoreCommand =new ScoreL2Superstructure();
                            raiseElevatorCommand1 = new QueueL2Action();
                            raiseElevatorCommand2 = new QueueL2Action();

                            break;
    
                        case "L3":
                            scoreCommand = new ScoreL3Superstructure();
                            raiseElevatorCommand1 = new QueueL3Action();
                            raiseElevatorCommand2 = new QueueL3Action();

                            break;
    
                        case "L4":
                            scoreCommand =new ScoreL4Superstructure();
                            raiseElevatorCommand1 = new QueueL4Action();
                            raiseElevatorCommand2 = new QueueL4Action();

                            break;
    
                        default:
                            throw new IOException("auto generator can not parse level");
                    }
                } 
                catch (Exception e) {
                    DriverStation.reportError("Oh shit, ur fucked haha " + e.getMessage(), e.getStackTrace());
                    return Commands.none();
                }

                if (i + 1 < params.length && params[i + 1].split(" ").length != 2 && params[i + 1] != "intake") {
                    try {
                        int index = commands.size() - 1;
                        commands.set(
                            index,
                            new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                    commands.get(index), // driving command,
                                    new SequentialCommandGroup(
                                        new WarmUpElevatorReef(),
                                        raiseElevatorCommand1
                                    )
                                ),
                                new SequentialCommandGroup(
                                    alignCommand,
                                    raiseElevatorCommand2,
                                    new EjectCoral()
                                )
                            )
                        );
                        commands.add(
                            new ParallelCommandGroup(
                                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(params[i + 1])),
                                new QueueStowAction()
                            )   
                        );
                        i++;
                    } 
                    catch (Exception e) {
                        DriverStation.reportError("Oh shit, ur fucked haha " + e.getMessage(), e.getStackTrace());
                        return Commands.none();
                    }
                }
                else {
                    int index = commands.size() - 1;
                    commands.set(
                        index,
                        new SequentialCommandGroup(
                            new ParallelDeadlineGroup(
                                commands.get(index), // driving command,
                                new SequentialCommandGroup(
                                    new WarmUpElevatorReef(),
                                    raiseElevatorCommand1
                                )
                            ),
                            new SequentialCommandGroup(
                                alignCommand,
                                scoreCommand
                            )
                        )
                    );
                }
            }

            // else if (param == "intake") {

            // }
            
            else {
                try {
                    commands.add(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(param)));
                } 
                catch (Exception e) {
                    DriverStation.reportError("Oh shit, ur fucked haha " + e.getMessage(), e.getStackTrace());
                    return Commands.none();
                }
            }
        }

        for (Command command : commands) {
            auto.addCommands(command);
        }
        return auto;
    }
}
