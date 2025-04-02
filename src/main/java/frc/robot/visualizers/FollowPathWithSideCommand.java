package frc.robot.visualizers;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;
import java.util.function.Consumer;

public class FollowPathWithSideCommand extends Command {

	private final Consumer<Pair<Translation2d, Boolean>> setPositionWithSide;
	private final List<Pair<Translation2d, Boolean>> path;

	private int currentPoint = 0;

	public FollowPathWithSideCommand(List<Pair<Translation2d, Boolean>> path, Consumer<Pair<Translation2d, Boolean>> setPositionWithSide) {
		this.path = path;
		this.setPositionWithSide = setPositionWithSide;
	}

	@Override
	public void initialize() {
		currentPoint = 0;
	}

	@Override
	public void execute() {
		setPositionWithSide.accept(path.get(currentPoint));
		currentPoint++;
	}

	@Override
	public boolean isFinished() {
		return currentPoint >= path.size();
	}

}
