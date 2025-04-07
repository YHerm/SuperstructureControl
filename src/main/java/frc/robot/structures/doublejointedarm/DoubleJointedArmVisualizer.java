package frc.robot.structures.doublejointedarm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.ArrayList;
import java.util.List;

public class DoubleJointedArmVisualizer {

	private static final MechanismLigament2d TARGET_POSITION_MARK = new MechanismLigament2d("mark", 0.01, 0, 10.0F, new Color8Bit(Color.kGreen));
	private static final double DEFAULT_LINE_WIDTH = 10.0F;

	private final Translation2d armRootPosition;

	private final Mechanism2d mechanism;
	private final MechanismRoot2d root;
	private final MechanismLigament2d elbow;
	private final MechanismLigament2d wrist;
	private final MechanismRoot2d targetPosition;
	private final ArrayList<ArrayList<MechanismLigament2d>> paths = new ArrayList<>();

	private int pathCounter = 0;

	public DoubleJointedArmVisualizer(
		String name,
		double frameXMeters,
		double frameYMeters,
		double elbowLengthMeters,
		double wristLengthMeters
	) {
		this.mechanism = new Mechanism2d(frameXMeters, frameYMeters);
		this.armRootPosition = new Translation2d(frameXMeters / 2.0, 0);
		this.root = mechanism.getRoot(name + " DoubleJointedArm", frameXMeters / 2.0, 0);
		this.elbow = new MechanismLigament2d("elbow", elbowLengthMeters, 0);
		this.wrist = new MechanismLigament2d("wrist", wristLengthMeters, 0, DEFAULT_LINE_WIDTH, new Color8Bit(Color.kPurple));
		this.targetPosition = mechanism.getRoot(name + " TargetPosition", 0, 0);

		elbow.append(wrist);
		root.append(elbow);
		targetPosition.append(TARGET_POSITION_MARK);

		showTargetPosition(false);

		SmartDashboard.putData(name + " DoubleJointedArmMech2d", mechanism);
	}

	public void setAngles(Rotation2d firstJointAngle, Rotation2d secondJointAngle) {
		elbow.setAngle(firstJointAngle);
		wrist.setAngle(toFloorRelative(firstJointAngle, secondJointAngle));
	}

	public void setTargetPositionMeters(Translation2d positionMeters) {
		targetPosition.setPosition(positionMeters.getX() + armRootPosition.getX(), positionMeters.getY() + armRootPosition.getY());
		showTargetPosition(true);
	}

	public void showTargetPosition(boolean show) {
		TARGET_POSITION_MARK.setLineWeight(show ? DEFAULT_LINE_WIDTH : 0);
	}

	public void showPath(List<Trajectory.State> path) {
		if (!paths.isEmpty()) {
			for (int i = 0; i < paths.get(0).size(); i++) {
				paths.get(0).get(i).setLineWeight(0);
				paths.get(0).get(i).close();
			}
			paths.remove(0);
		}

		ArrayList<MechanismLigament2d> pathVisualize = new ArrayList<>(path.size());
		for (int i = 0; i < path.size(); i++) {
			MechanismRoot2d root = mechanism.getRoot(
				pathCounter + ", " + i,
				path.get(i).poseMeters.getX() + armRootPosition.getX(),
				path.get(i).poseMeters.getY() + armRootPosition.getY()
			);
			MechanismLigament2d mark = new MechanismLigament2d(
				pathCounter + ", " + i + " mark",
				0,
				0,
				DEFAULT_LINE_WIDTH,
				new Color8Bit(Color.kCyan)
			);
			pathVisualize.add(mark);
			root.append(mark);
		}
		paths.add(pathVisualize);
		pathCounter++;
	}

	private static Rotation2d toFloorRelative(Rotation2d floorRelativeAngle, Rotation2d jointRelativeAngle) {
		return jointRelativeAngle.minus(floorRelativeAngle);
	}

}
