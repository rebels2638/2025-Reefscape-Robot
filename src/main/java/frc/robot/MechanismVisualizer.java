package frc.robot;

import java.util.ArrayList;
import java.util.Map;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climber.simple.MoveDeepCage;
import frc.robot.constants.Constants;
import frc.robot.constants.MechAElementConstants;
import frc.robot.constants.MechAElementConstants.ReefHeight;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller;

public class MechanismVisualizer extends SubsystemBase {
    private static MechanismVisualizer instance;
    public static MechanismVisualizer getInstance() {
        if (instance == null) {
            instance = new MechanismVisualizer();
        }
        return instance;
    }

    private final Elevator elevator = Elevator.getInstance();
    private final Pivot pivot = Pivot.getInstance();
    private final Roller roller = Roller.getInstance();
    private final Climber climber = Climber.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    @AutoLogOutput
    private final LoggedMechanism2d superstructureMech = new LoggedMechanism2d(0.83, 3);

    private final LoggedMechanismRoot2d superstructureRoot = 
        superstructureMech.getRoot(
            "superstructure", 0.55, 0 
        );//0.415, 0.076

    private final LoggedMechanismLigament2d elevatorLigament = 
        superstructureRoot.append(
            new LoggedMechanismLigament2d("elevator", 0, 90)
        );
    
    private final LoggedMechanismLigament2d offsetLigament = 
        elevatorLigament.append(
            new LoggedMechanismLigament2d("offset", 0.1, 0, 6, new Color8Bit(Color.kBlack))
        );

    private final LoggedMechanismLigament2d pivotLigament = 
        offsetLigament.append(
            new LoggedMechanismLigament2d("pivot", 0.4, 90, 6, new Color8Bit(Color.kPurple))
        );

    private final LoggedMechanismLigament2d climberLigament = 
        offsetLigament.append(
            new LoggedMechanismLigament2d("climber", 0.3, 74.0, 6, new Color8Bit(Color.kBlue))
        );
    
    private ArrayList<Pose3d> coralPoses = new ArrayList<Pose3d>(0);

    private boolean inRoller = roller.inRoller();

    private Rotation3d rpRot = new Rotation3d(0.0, 0.0, robotState.getEstimatedPose().getRotation().getRadians());
    private Translation3d rpTranslation = new Translation3d(robotState.getEstimatedPose().getX(), 
                                                        robotState.getEstimatedPose().getY(), 0.0);

    @Override
    public void periodic() {
        elevatorLigament.setLength(elevator.getHeight() + 0.45);
        pivotLigament.setAngle(-pivot.getAngle().getDegrees());
        climberLigament.setAngle(-climber.getAngle().getDegrees());
        
        Pose2d rp = robotState.getEstimatedPose();

        Pose3d robotPose = new Pose3d(
            new Translation3d(
                rpTranslation.getX(),
                rpTranslation.getY(),
                rpTranslation.getZ()
            ),
            new Rotation3d(
                rpRot.getX(),
                rpRot.getY(),
                rpRot.getZ()
            )
        );

        Transform3d rollerTransform = 
            new Transform3d(
                new Translation3d(
                    0.23,
                    0.0,
                    elevatorLigament.getLength() + robotPose.getZ() + 0.05
                ),
                new Rotation3d(
                    0.0,
                    0.0,
                    0.0
                )
            );

        Pose3d rollerPose = robotPose.transformBy(rollerTransform);

        if (roller.inRoller()) {
            Logger.recordOutput("MechanismVisualizer/rollerPose", rollerPose);
        }
        else {
            Logger.recordOutput("MechanismVisualizer/rollerPose", new Pose3d());
        }

        Pose3d algayPose = new Pose3d(rollerPose.getX(), rollerPose.getY(), rollerPose.getZ() + 0.3, new Rotation3d());
        if (roller.inRoller()) {
            Logger.recordOutput("MechanismVisualizer/clawPose", algayPose);
        }
        else {
            Logger.recordOutput("MechanismVisualizer/clawPose", new Pose3d());
        }

        if (!roller.inRoller() && inRoller) {
            Pose3d nearest = new Pose3d();
            for (Map<ReefHeight, Pose3d> coralPose : MechAElementConstants.Reef.branchPositions) {
                for (Pose3d coral : coralPose.values()) {
                    coral = 
                        Constants.shouldFlipPath() ? 
                            new Pose3d(
                                FlippingUtil.flipFieldPose(coral.toPose2d()).getX(),
                                FlippingUtil.flipFieldPose(coral.toPose2d()).getY(),
                                coral.getTranslation().getZ(),
                                new Rotation3d(
                                    coral.getRotation().getX(),
                                    coral.getRotation().getY(),
                                    FlippingUtil.flipFieldPose(coral.toPose2d()).getRotation().getRadians()
                                )
                            ) :
                            coral;

                    if (coral.getTranslation().getDistance(rollerPose.getTranslation()) < 
                        nearest.getTranslation().getDistance(rollerPose.getTranslation()) &&
                        coral.getTranslation().getDistance(rollerPose.getTranslation()) < 0.5) {
                        nearest = coral;
                    }
                }
            }
            coralPoses.add(nearest);
        }

        inRoller = roller.inRoller();

        Pose3d[] coralPoseArray = new Pose3d[coralPoses.size()];
        coralPoseArray = coralPoses.toArray(coralPoseArray);

        Logger.recordOutput("MechanismVisualizer/coralPoses", coralPoseArray);

        Transform3d climberTransform = 
            new Transform3d(
                new Translation3d(
                    0.225,
                    0.225,
                    climberLigament.getLength() + robotPose.getZ() + 0.05
                ),
                new Rotation3d(
                    0.0,
                    climber.getAngle().getRadians(),
                    Units.degreesToRadians(30.0) + rpRot.getZ()
                )
            );

        Pose3d climberPose = robotPose.transformBy(climberTransform);

        if (climberPose != null) {
            Logger.recordOutput("MechanismVisualizer/climberPose", climberPose);
        }
        else {
            Logger.recordOutput("MechanismVisualizer/climberPose", new Pose3d());
        }

        // Logger.recordOutput("MechanismVisualizer/climberPose", climberPose);

        Translation2d cage = new Translation2d();
        Translation2d prevCage = MechAElementConstants.Barge.cages[0];

        for (int i = 0; i < MechAElementConstants.Barge.cages.length; i++) {
            if (new Translation2d(robotPose.getX(), robotPose.getY()).getDistance(prevCage) >= new Translation2d(robotPose.getX(), 
                robotPose.getY()).getDistance(MechAElementConstants.Barge.cages[i])) {

                cage = MechAElementConstants.Barge.cages[i];
            }
        }

        if (!climber.reachedSetpoint()) { //idk the condition is probably different but wtv it works
            if (new Translation2d(cage.getX(), cage.getY()).getDistance(new Translation2d(robotPose.getX(), robotPose.getY())) <= 0.4 && 
                Math.abs(Units.degreesToRadians(30.0) - rp.getRotation().getRadians()) <= Units.degreesToRadians(10.0) && 
                Math.abs(climber.getAngle().minus(Rotation2d.fromDegrees(130.0)).getDegrees()) <= 5.0) {

                rpTranslation = new Translation3d(
                    cage.getX(),
                    cage.getY(),
                    cage.getY() == MechAElementConstants.Barge.cages[1].getY() ?
                                     MechAElementConstants.Barge.shallowHeight : //shallow is SO FUCKED LMAO (if u can fix pls fix)
                                     MechAElementConstants.Barge.deepHeight
                );
                rpRot = new Rotation3d(
                    0.0,
                    0.0,
                    120.0
                );
            }
            else {
                rpTranslation = new Translation3d(
                    rp.getX(),
                    rp.getY(),
                    0.0
                );
                rpRot = new Rotation3d(
                    0.0,
                    0.0,
                    rp.getRotation().getRadians()
                );
            }
        }
        
        Logger.recordOutput("MechanismVisualizer/robotPose", robotPose);
    }
}
