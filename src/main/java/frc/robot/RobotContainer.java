package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.AutoRunner;
import frc.robot.commands.autoAlignment.LinearDriveToPose;
import frc.robot.commands.autoAlignment.LockDriveAxis;
import frc.robot.commands.autoAlignment.PathPlanToPose;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    public static RobotContainer instance = null;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }

        return instance;
    }

    

    private final SwerveDrive swerveDrive;
    private final AutoRunner autoRunner;
    private final Vision vision;

    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;
    
    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        swerveDrive = SwerveDrive.getInstance();
        vision = Vision.getInstance();
        autoRunner = AutoRunner.getInstance();

        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));

        // xboxOperator.getAButton().onTrue(new IntakeCoral());
        // xboxOperator.getBButton().onTrue(new EjectCoral());

        // xboxDriver.getAButton()
        //         .whileTrue(new LinearDriveToPose(new Pose2d(5, 5, new Rotation2d(3)), new ChassisSpeeds(0, 0, 0)));
        xboxDriver.getAButton().whileTrue(new LockDriveAxis(xboxDriver));
        // xboxDriver.getYButton().onTrue(new PathPlanToPose(PathPlanToPose.alignmentPoseSearch()));

        // xboxDriver.getXButton().onTrue(new InstantCommand(() -> RobotState.getInstance().zeroGyro()));
    }

    public Command getAutonomousCommand() {
        return (Command) null;
    }

}
