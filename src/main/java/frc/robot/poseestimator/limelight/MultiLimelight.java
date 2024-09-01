package frc.robot.poseestimator.limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Robot;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;


public class MultiLimelight extends GBSubsystem {

    private List<Limelight> limelights;

    private MultiLimelight() {
        super(VisionConstants.MultiLimeLightLogPath);
        limelights = new ArrayList<>();
        for (String limelightName : VisionConstants.LIMELIGHT_NAMES) {
            limelights.add(new Limelight(limelightName));
        }
    }

    public List<Optional<Pair<Pose2d, Double>>> getAll2DEstimates() {
        ArrayList<Optional<Pair<Pose2d, Double>>> estimates = new ArrayList<>();

        for (Limelight limelight : limelights) {
            if (isLimelightValid(limelight)) {
                estimates.add(limelight.getUpdatedPose2DEstimation());
            }
        }

        return estimates;
    }

    public boolean isLimelightedOutputInTolerance(Limelight limelight) {
        Pose2d limelightPosition;
        Transform2d transformDifference;
        Rotation2d rotationDifference;

        Pose2d currentPosition = Robot.getPosEstimator.getOdometryPose();

        limelightPosition = limelight.getUpdatedPose2DEstimation().get().getFirst();
        transformDifference = limelightPosition.minus(currentPosition);
        rotationDifference = limelightPosition.getRotation().minus(currentPosition.getRotation());

        return transformDifference.getTranslation().getNorm() <= VisionConstants.POSITION_TOLERANCE
                && rotationDifference.getDegrees() <= VisionConstants.ROTATION_TOLERANCE.getDegrees();
    }

    public boolean isLimelightValid(Limelight limelight) {
        if (limelight.hasTarget()) {
            if (limelight.isAprilTagInProperHeight())
                if (limelight.getUpdatedPose2DEstimation().isPresent()) {
                    return isLimelightedOutputInTolerance(limelight);
                }
        }

        return false;
    }

    public void recordEstimatedPositions() {
        int i = 0;
        for (Optional<Pair<Pose2d, Double>> estimation : getAll2DEstimates()) {
            i++;
            if (estimation.isPresent()) {
                Logger.recordOutput(super.getLogPath() + VisionConstants.EstimationLogPath + i, estimation.get().getFirst());
            }
        }
    }

    public double getDynamicStandardDeviations(int limelightId) {
        return limelights.get(limelightId).getDistanceFromAprilTag() / VisionConstants.VISION_TO_STANDARD_DEVIATION;
    }

    public boolean hasTarget(int limelightId) {
        return limelights.get(limelightId).hasTarget();
    }

    public Optional<Pair<Pose2d, Double>> getFirstAvailableTarget() {
        for (Optional<Pair<Pose2d, Double>> output : getAll2DEstimates()) {
            if (output.isPresent()) {
                return output;
            }
        }

        return Optional.empty();
    }

    public boolean isConnected() {
        return limelights.get(0).hasTarget();
    }

    @Override
    protected void subsystemPeriodic() {

    }

}
