package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.alge.AlgayControllerRaw;
import frc.robot.commands.coralRoller.RollerRun;
import frc.robot.commands.coralRoller.RollerStop;
import frc.robot.commands.drivetrain.AbsoluteFieldDrive;
import frc.robot.commands.elevator.ElevatorControlRaw;
import frc.robot.commands.elevator.MoveElevatorToggle;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOFalcon;
import frc.robot.subsystems.roller.Roller;

public class RobotContainer {
  public static RobotContainer instance = null;

  private final XboxController xboxTester;
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;
  
  private final SwerveDrive swerveDrive;
  private final Elevator elevator;
  private final Roller roller;

  public RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    swerveDrive = new SwerveDrive();
    elevator = new Elevator(new ElevatorIOFalcon());
    roller = new Roller();

    // elevator.setDefaultCommand(new ElevatorControlRaw(xboxOperator, elevator));
    // xboxOperator.getXButton().onTrue(new RollerRun(roller));
    // xboxOperator.getAButton().onTrue(new RollerStop(roller));
    // this.xboxDriver.getAButton().onTrue((new MoveElevatorToggle()));

    roller.setDefaultCommand(new AlgayControllerRaw(xboxOperator, roller));

    // swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(swerveDrive, xboxDriver));
    xboxDriver.getXButton().onTrue(new InstantCommand(() -> swerveDrive.zeroGyro()));
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

