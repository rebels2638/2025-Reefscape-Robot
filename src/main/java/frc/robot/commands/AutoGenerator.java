package frc.robot.commands;

import java.io.IOException;
import java.util.HashMap;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.autoAlignment.reef.AlignToLeftBranch;
import frc.robot.commands.autoAlignment.reef.AlignToRightBranch;
import frc.robot.commands.complex.superstructure.ScoreL1Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL2Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL3Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL4Superstructure;
import frc.robot.commands.roller.IntakeCoral;
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
     *  - "intake" -> will schedule a intake command. 
     *    this could be passed before or after a source going path, depending on desired functionality. ex. -> "intake"
     */
    public static Command generateAuto(String... params) {
        SequentialCommandGroup auto = new SequentialCommandGroup();

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
        

        for (int i = 1; i < params.length; i++) {
            String param = params[i];

            if (param == "intake") {
                auto.addCommands(new IntakeCoral());
            }
            else if (param.split(" ").length == 2) {
                try {
                    String[] split = param.split(" ");
                    switch (split[0]) { // auto align because we need maximal accuracy
                        case "RB":
                            auto.addCommands(new AlignToRightBranch());
                            break;
                    
                        case "LB":
                            auto.addCommands(new AlignToLeftBranch());
                            break;
    
                        default:
                            throw new IOException("auto generator can not parse branch");
                    }
    
                    switch (split[1]) {
                        case "L1":
                            auto.addCommands(new ScoreL1Superstructure());
                            break;
                    
                        case "L2":
                            auto.addCommands(new ScoreL2Superstructure());
                            break;
    
                        case "L3":
                            auto.addCommands(new ScoreL3Superstructure());
                            break;
    
                        case "L4":
                            auto.addCommands(new ScoreL4Superstructure());
                            break;
    
                        default:
                            throw new IOException("auto generator can not parse level");
                    }
                } 
                catch (Exception e) {
                    DriverStation.reportError("Oh shit, ur fucked haha " + e.getMessage(), e.getStackTrace());
                    return Commands.none();
                }
                
            }

            else if (param == "intake") {
                auto.addCommands(new IntakeCoral()); // We don't want to auto align here as we can rely on the auto to have good enough accuracy
            }
            
            else {
                try {
                    auto.addCommands(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(param)));
                } 
                catch (Exception e) {
                    DriverStation.reportError("Oh shit, ur fucked haha " + e.getMessage(), e.getStackTrace());
                    return Commands.none();
                }
            }
        }

        return auto;
    }
}
