package frc.robot.visualizers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class DoubleJointedArmVisualizer {

	private final Mechanism2d mechanism;
	private final MechanismRoot2d root;
	private final MechanismLigament2d firstJoint;
	private final MechanismLigament2d secondJoint;

	public DoubleJointedArmVisualizer(
		String name,
		double frameXMeters,
		double frameYMeters,
		double firstJointLengthMeters,
		double secondJointLengthMeters
	) {
		this.mechanism = new Mechanism2d(frameXMeters, frameYMeters);
		this.root = mechanism.getRoot(name + " DoubleJointedArm", frameXMeters / 2.0, 0);
		this.firstJoint = new MechanismLigament2d("first joint", firstJointLengthMeters, 0);
		this.secondJoint = new MechanismLigament2d("second joint", secondJointLengthMeters, 0, 10.0F, new Color8Bit(Color.kPurple));

		firstJoint.append(secondJoint);
		root.append(firstJoint);
		SmartDashboard.putData(name + " DoubleJointedArmMech2d", mechanism);
	}

	public void setAngles(Rotation2d firstJointAngle, Rotation2d secondJointAngle) {
		firstJoint.setAngle(firstJointAngle);
		secondJoint.setAngle(toFloorRelative(firstJointAngle, secondJointAngle));
	}

	private static Rotation2d toFloorRelative(Rotation2d floorRelativeAngle, Rotation2d jointRelativeAngle) {
		return jointRelativeAngle.minus(floorRelativeAngle);
	}

}
