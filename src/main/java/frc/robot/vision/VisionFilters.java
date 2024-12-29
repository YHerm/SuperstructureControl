package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.Field;
import frc.constants.MathConstants;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.Filter;
import frc.utils.ToleranceUtils;

public class VisionFilters {

	public static Filter<VisionData> isPitchAtAngle(Rotation2d wantedPitch, Rotation2d pitchTolerance) {
		return new Filter<>(
			visionData -> MathUtil.isNear(
				wantedPitch.getRadians(),
				visionData.getEstimatedPose().getRotation().getY(),
				pitchTolerance.getRadians(),
				0,
				MathConstants.FULL_CIRCLE.getRadians()
			)
		);
	}

	public static Filter<VisionData> isRollAtAngle(Rotation2d wantedRoll, Rotation2d rollTolerance) {
		return new Filter<>(
			visionData -> MathUtil.isNear(
				wantedRoll.getRadians(),
				visionData.getEstimatedPose().getRotation().getX(),
				rollTolerance.getRadians(),
				0,
				MathConstants.FULL_CIRCLE.getRadians()
			)
		);
	}

	public static Filter<VisionData> isOnGround(double distanceFromGroundToleranceMeters) {
		return new Filter<>(visionData -> MathUtil.isNear(0, visionData.getEstimatedPose().getZ(), distanceFromGroundToleranceMeters));
	}

	public static Filter<AprilTagVisionData> isAprilTagHeightValid(double aprilTagRealHeightMeters, double aprilTagHeightToleranceMeters) {
		return new Filter<>(
			aprilTagVisionData -> Math.abs(aprilTagVisionData.getAprilTagHeightMeters() - aprilTagRealHeightMeters)
				<= aprilTagHeightToleranceMeters
		);
	}

	public static Filter<VisionData> isXInField(double xToleranceMeters) {
		return new Filter<>(
			visionData -> ToleranceUtils.isInRange(xToleranceMeters, visionData.getEstimatedPose().getX(), Field.LENGTH_METERS, 0)
		);
	}

	public static Filter<VisionData> isYInField(double yToleranceMeters) {
		return new Filter<>(
			visionData -> ToleranceUtils.isInRange(yToleranceMeters, visionData.getEstimatedPose().getY(), Field.WIDTH_METERS, 0)
		);
	}

	public static Filter<VisionData> isInField(double positionToleranceMeters) {
		return isXInField(positionToleranceMeters).and(isYInField(positionToleranceMeters));
	}

}
