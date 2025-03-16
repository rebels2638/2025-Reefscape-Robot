package frc.robot.commands.climber.simple;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.pneumatics.Pneumatics;

public class EnableOppositeRotation extends ConditionalCommand {
    public EnableOppositeRotation() {
        super(
            new InstantCommand(() -> Pneumatics.getInstance().pushRatchet()), 
            new InstantCommand(() -> Pneumatics.getInstance().pullRatchet()), 
            () -> Pneumatics.getInstance().getRatchetState()
        );
    }
}
