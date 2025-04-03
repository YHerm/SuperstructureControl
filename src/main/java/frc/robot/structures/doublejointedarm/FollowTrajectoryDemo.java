package frc.robot.structures.doublejointedarm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.time.TimeUtil;

import java.util.function.Consumer;

public class FollowTrajectoryDemo extends Command {

	private final Consumer<Translation2d> setPosition;
	private final Trajectory path;

	private double startTime = 0;

	public FollowTrajectoryDemo(Trajectory trajectory, Consumer<Translation2d> setPosition) {
		this.path = trajectory;
		this.setPosition = setPosition;
	}

	@Override
	public void initialize() {
		startTime = TimeUtil.getCurrentTimeSeconds();
	}

	@Override
	public void execute() {
		setPosition.accept(path.sample(TimeUtil.getCurrentTimeSeconds() - startTime).poseMeters.getTranslation());
	}

	@Override
	public boolean isFinished() {
		return TimeUtil.getCurrentTimeSeconds() - startTime >= path.getTotalTimeSeconds();
	}

}
