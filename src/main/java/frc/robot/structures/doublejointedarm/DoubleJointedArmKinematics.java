package frc.robot.structures.doublejointedarm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;

public class DoubleJointedArmKinematics {

	public static Translation2d toPositionMeters(ArmAngles angles, double firstJointLengthMeters, double secondJointLengthMeters) {
		return new Translation2d(
			firstJointLengthMeters * angles.firstJoint().getCos() + secondJointLengthMeters * angles.secondJoint().getCos(),
			firstJointLengthMeters * angles.firstJoint().getSin() + secondJointLengthMeters * angles.secondJoint().getSin()
		);
	}
	
	public static Optional<ArmAngles> toAngles(Translation2d positionMeters, double firstJointLengthMeters, double secondJointLengthMeters, boolean elbowLeft) {
		double x = positionMeters.getX();
		double y = positionMeters.getY();

		Optional<Rotation2d> theta2Optional = cosineLaw(Math.sqrt(x * x + y * y), firstJointLengthMeters, secondJointLengthMeters);
		if (theta2Optional.isEmpty()) {
			return Optional.empty();
		}
		
		double wristAngleRads = theta2Optional.get().getRadians();

		if (elbowLeft) {
			wristAngleRads = -wristAngleRads;
		}

		double elbowAngleRads = Math.atan2(y, x)
				- Math
				.atan2(secondJointLengthMeters * Math.sin(wristAngleRads), firstJointLengthMeters + secondJointLengthMeters * Math.cos(wristAngleRads));

		return Optional.of(new ArmAngles(Rotation2d.fromRadians(elbowAngleRads), Rotation2d.fromRadians(elbowAngleRads + wristAngleRads)));
	}


	public static Optional<Rotation2d> cosineLaw(double farSide, double nearSide1, double nearSide2) {
		double cosTheta = (farSide * farSide - nearSide1 * nearSide1 - nearSide2 * nearSide2) / (2 * nearSide1 * nearSide2);

		if (cosTheta < -1 || cosTheta > 1) {
			return Optional.empty();
		}

		return Optional.of(Rotation2d.fromRadians(Math.acos(cosTheta)));
	}

}
