package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

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
}