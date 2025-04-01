package frc.robot.visualizers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.time.TimeUtil;

import java.util.function.Consumer;

public class FollowPathCommand extends Command {

	private final Consumer<Translation2d> setPosition;
	private final double delayBetweenPoints;
	private final Translation2d[] points;

	private double setPositionTime = 0;
	private int currentPoint = 1;

	public FollowPathCommand(Consumer<Translation2d> setPosition, double delayBetweenPoints, Translation2d... points) {
		this.setPosition = setPosition;
		this.delayBetweenPoints = delayBetweenPoints;
		this.points = points;
	}

	@Override
	public void initialize() {
		setPositionTime = TimeUtil.getCurrentTimeSeconds();
		currentPoint = 1;
	}

	@Override
	public void execute() {
		if (TimeUtil.getCurrentTimeSeconds() - setPositionTime >= delayBetweenPoints) {
			setPosition.accept(points[currentPoint]);
			currentPoint++;
			setPositionTime = TimeUtil.getCurrentTimeSeconds();
		}
	}

	@Override
	public boolean isFinished() {
		return currentPoint >= points.length;
	}

}
