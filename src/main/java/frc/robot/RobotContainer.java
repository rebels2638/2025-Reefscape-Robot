package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.claw.IntakeAlgay;
import frc.robot.commands.claw.simple.StopClaw;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.superstructure.Superstructure;
  
public class RobotContainer {
  public static RobotContainer instance = null;

  private final XboxController xboxTester;
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;
  
  private final SwerveDrive swerveDrive;
  
  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    swerveDrive = new SwerveDrive();
    swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(swerveDrive, xboxDriver));

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

