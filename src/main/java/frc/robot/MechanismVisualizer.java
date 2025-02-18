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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.MechAElementConstants;
import frc.robot.constants.MechAElementConstants.ReefHeight;
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
    
    private ArrayList<Pose3d> coralPoses = new ArrayList<Pose3d>(0);

    private boolean inRoller = roller.inRoller();

    @Override
    public void periodic() {
        elevatorLigament.setLength(elevator.getHeight() + 0.45);
        pivotLigament.setAngle(-pivot.getAngle().getDegrees());

        
        Pose2d rp = robotState.getEstimatedPose();
        Pose3d robotPose = new Pose3d(
            new Translation3d(
                rp.getX(),
                rp.getY(),
                0
            ),
            new Rotation3d(
                0,
                0,
                rp.getRotation().getRadians()
            )
        );

        Transform3d rollerTransform = 
            new Transform3d(
                new Translation3d(
                    0.23,
                    0,
                    elevatorLigament.getLength() + 0.05
                ),
                new Rotation3d(
                    0,
                    0,
                    0
                )
            );

        Pose3d rollerPose = robotPose.transformBy(rollerTransform);

        if (roller.inRoller()) {
            Logger.recordOutput("MechanismVisualizer/rollerPose", rollerPose);
        }
        else {
            Logger.recordOutput("MechanismVisualizer/rollerPose", new Pose3d());
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
                                coral.getRotation()
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

    }
}
