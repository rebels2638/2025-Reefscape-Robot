package frc.robot.commands;

import java.io.SequenceInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;

public final class Autos {
    private Autos() {}

    public static final Command test2 = 
        AutoGenerator.generateAuto(
            "fromProcessorStartToTopRightRB", 
            "RB L4",
            "fromTopRightRBToRightSourceTop",
            "fromRightSourceTopToBottomRightRB",
            "RB L4"
        );
    
        public static final Command test4 = 
            AutoGenerator.generateAuto(
                "fromProcessorStartToTopRightLB", 
                "LB L4",
                "fromTopRightLBToRightSourceTop",
                "fromRightSourceTopToBottomRightLB",
                "LB L4",
                "fromBottomRightLBToRightSourceTop",
                "fromRightSourceTopToBottomRightRB",
                "RB L4",
                "fromBottomRightRBToRightSourceTop",
                "fromRightSourceTopToTopRightRB",
                "RB L4"
            );
        
        public static final Command test5 = 
            AutoGenerator.generateAuto(
                "fromProcessorStartToTopRightLB", 
                "LB L3",
                "fromTopRightLBToRightSourceTop",
                "fromRightSourceTopToBottomRightLB",
                "LB L3",
                "fromBottomRightLBToRightSourceTop",
                "fromRightSourceTopToBottomRightRB",
                "RB L3",
                "fromBottomRightRBToRightSourceTop",
                "fromRightSourceTopToTopRightRB",
                "RB L3"
            );


    public static Command getAutonomousCommand(String name) {
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);

            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().resetPose(path.getStartingHolonomicPose().get())),
                AutoBuilder.followPath(path)
            );

        } catch (Exception e) {
            DriverStation.reportError("Oh shit, ur fucked haha " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
              
    }
}