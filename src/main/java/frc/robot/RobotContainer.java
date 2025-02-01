package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.commands.roller.IntakeCoral;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
  
public class RobotContainer {
  public static RobotContainer instance = null;

  private final XboxController xboxTester;
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;
  
  private final SwerveDrive swerveDrive;
  private final Vision vision;
  // private final Roller roller;

  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    swerveDrive = SwerveDrive.getInstance();
    vision = Vision.getInstance();
    // roller = Roller.getInstance();

    swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));

    // xboxOperator.getAButton().onTrue(new IntakeCoral());
    // xboxOperator.getBButton().onTrue(new EjectCoral());
    
    xboxDriver.getXButton().onTrue(new InstantCommand(() -> RobotState.getInstance().zeroGyro()));
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }
  
  public Command getAutonomousCommand() {
    return (Command) null;
  }

}

