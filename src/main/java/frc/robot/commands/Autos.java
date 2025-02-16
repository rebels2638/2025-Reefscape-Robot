package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {
    private Autos() {}

    public static final Command test = 
        AutoGenerator.generateAuto(
            "fromProcessorStartToTopRightRB", 
            "RB L4",
            "fromTopRightRBToRightSourceTop",
            "fromRightSourceTopToBottomRightRB",
            "RB L4"
        );
}