package frc.robot.visualizers;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class PathGenerator {


	public static List<Translation2d> straightLine(Translation2d start, Translation2d end, int steps) {
		List<Translation2d> points = new ArrayList<>();

		double deltaX = end.getX() - start.getX();
		double deltaY = end.getY() - start.getY();

		double stepX = deltaX / (steps - 1);
		double stepY = deltaY / (steps - 1);

		for (int i = 0; i < steps; i++) {
			double x = start.getX() + i * stepX;
			double y = start.getY() + i * stepY;
			points.add(new Translation2d(x, y));
		}

		return points;
	}

}
