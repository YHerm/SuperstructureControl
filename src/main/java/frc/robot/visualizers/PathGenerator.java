package frc.robot.visualizers;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class PathGenerator {


	public static List<Translation2d> straightLine(Translation2d start, Translation2d end) {
		int steps = (int) (end.getDistance(start) / 0.015);
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

	public static List<Pair<Translation2d, Boolean>> straightLine(Translation2d start, Translation2d end, boolean isLeft) {
		int steps = (int) (end.getDistance(start) / 0.015);
		List<Pair<Translation2d, Boolean>> points = new ArrayList<>();

		double deltaX = end.getX() - start.getX();
		double deltaY = end.getY() - start.getY();

		double stepX = deltaX / (steps - 1);
		double stepY = deltaY / (steps - 1);

		for (int i = 0; i < steps; i++) {
			double x = start.getX() + i * stepX;
			double y = start.getY() + i * stepY;
			points.add(new Pair<>(new Translation2d(x, y), isLeft));
		}

		return points;
	}

	public static List<Pair<Translation2d, Boolean>> flipArm(Translation2d point, boolean isStartLeft, double length) {
		Translation2d fullExtendedPoint = new Translation2d(point.getAngle().getCos() * length, point.getAngle().getSin() * length);
		List<Pair<Translation2d, Boolean>> firstPath = straightLine(point, fullExtendedPoint, isStartLeft);
		List<Pair<Translation2d, Boolean>> secondPath = straightLine(fullExtendedPoint, point, !isStartLeft);

		firstPath.addAll(secondPath);
		return firstPath;
	}

	public static List<Translation2d> createTranslation2dList(List<Pair<Translation2d, Boolean>> flipArm) {
		List<Translation2d> translationList = new ArrayList<>();

		for (Pair<Translation2d, Boolean> pair : flipArm) {
			translationList.add(pair.getFirst());
		}

		return translationList;
	}

}
