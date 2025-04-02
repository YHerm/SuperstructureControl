package frc.robot.visualizers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.time.TimeUtil;

import java.util.List;
import java.util.function.Consumer;

public class FollowPathCommand extends Command {

	private final Consumer<Translation2d> setPosition;
	private final List<Translation2d> path;

	private int currentPoint = 0;

	public FollowPathCommand(List<Translation2d> path, Consumer<Translation2d> setPosition) {
		this.path = path;
		this.setPosition = setPosition;
	}

	@Override
	public void initialize() {
		currentPoint = 0;
	}

	@Override
	public void execute() {
		setPosition.accept(path.get(currentPoint));
		currentPoint++;
	}

	@Override
	public boolean isFinished() {
		return currentPoint >= path.size();
	}

}
