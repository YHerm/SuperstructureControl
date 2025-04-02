package frc.robot.visualizers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
	private final MechanismLigament2d firstJoint;
	private final MechanismLigament2d secondJoint;
	private final MechanismRoot2d targetPosition;
	private final ArrayList<ArrayList<MechanismLigament2d>> paths = new ArrayList<>();

	public DoubleJointedArmVisualizer(
		String name,
		double frameXMeters,
		double frameYMeters,
		double firstJointLengthMeters,
		double secondJointLengthMeters
	) {
		this.mechanism = new Mechanism2d(frameXMeters, frameYMeters);
		this.armRootPosition = new Translation2d(frameXMeters / 2.0, 0);
		this.root = mechanism.getRoot(name + " DoubleJointedArm", frameXMeters / 2.0, 0);
		this.firstJoint = new MechanismLigament2d("first joint", firstJointLengthMeters, 0);
		this.secondJoint = new MechanismLigament2d("second joint", secondJointLengthMeters, 0, DEFAULT_LINE_WIDTH, new Color8Bit(Color.kPurple));
		this.targetPosition = mechanism.getRoot(name + " TargetPosition", 0, 0);

		firstJoint.append(secondJoint);
		root.append(firstJoint);
		targetPosition.append(TARGET_POSITION_MARK);

		showTargetPosition(false);

		SmartDashboard.putData(name + " DoubleJointedArmMech2d", mechanism);
	}

	public void setAngles(Rotation2d firstJointAngle, Rotation2d secondJointAngle) {
		firstJoint.setAngle(firstJointAngle);
		secondJoint.setAngle(toFloorRelative(firstJointAngle, secondJointAngle));
	}

	public void setTargetPositionMeters(Translation2d positionMeters) {
		targetPosition.setPosition(positionMeters.getX() + armRootPosition.getX(), positionMeters.getY() + armRootPosition.getY());
		showTargetPosition(true);
	}

	public void showTargetPosition(boolean show) {
		TARGET_POSITION_MARK.setLength(show ? 0.01 : 0);
	}

	public void showPath(List<Translation2d> path) {
		ArrayList<MechanismLigament2d> pathVisualize = new ArrayList<>(path.size());
		for (int i = 0; i < path.size(); i++) {
			MechanismRoot2d root = mechanism
				.getRoot(paths.size() + ", " + i, path.get(i).getX() + armRootPosition.getX(), path.get(i).getY() + armRootPosition.getY());
			MechanismLigament2d mark = new MechanismLigament2d(
				paths.size() + ", " + i + " mark",
				0.01,
				0,
				10.0F,
				new Color8Bit(Color.kBlueViolet)
			);
			pathVisualize.add(mark);
			root.append(mark);
		}
		if (!paths.isEmpty()) {
			for (int i = 0; i < paths.get(paths.size() - 1).size(); i++) {
				paths.get(paths.size() - 1).get(i).setLength(0);
			}
		}
		paths.add(pathVisualize);
	}

	private static Rotation2d toFloorRelative(Rotation2d floorRelativeAngle, Rotation2d jointRelativeAngle) {
		return jointRelativeAngle.minus(floorRelativeAngle);
	}

}
