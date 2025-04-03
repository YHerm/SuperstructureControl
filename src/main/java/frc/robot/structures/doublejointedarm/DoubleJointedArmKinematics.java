package frc.robot.structures.doublejointedarm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;

public class DoubleJointedArmKinematics {

	public static Translation2d toPositionMeters(ArmAngles angles, double elbowLengthMeters, double wristLengthMeters) {
		return new Translation2d(
			elbowLengthMeters * angles.elbowAngle().getCos() + wristLengthMeters * angles.wristAngle().getCos(),
			elbowLengthMeters * angles.elbowAngle().getSin() + wristLengthMeters * angles.wristAngle().getSin()
		);
	}

	public static Optional<ArmAngles> toAngles(
		Translation2d positionMeters,
		double elbowLengthMeters,
		double wristLengthMeters,
		boolean isElbowLeft
	) {
		double x = positionMeters.getX();
		double y = positionMeters.getY();

		Optional<Rotation2d> wristAngleOptional = cosineLaw(Math.sqrt(x * x + y * y), elbowLengthMeters, wristLengthMeters);
		if (wristAngleOptional.isEmpty()) {
			return Optional.empty();
		}

		double wristAngleRads = wristAngleOptional.get().getRadians();

		if (isElbowLeft) {
			wristAngleRads = -wristAngleRads;
		}

		double elbowAngleRads = Math.atan2(y, x)
			- Math.atan2(wristLengthMeters * Math.sin(wristAngleRads), elbowLengthMeters + wristLengthMeters * Math.cos(wristAngleRads));

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
