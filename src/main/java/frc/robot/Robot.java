// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.DoubleJointedArm;
import frc.robot.visualizers.DoubleJointedArmVisualizer;
import frc.utils.battery.BatteryUtil;

import java.util.ArrayList;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final DoubleJointedArm arm = new DoubleJointedArm("Arm");
	private final DoubleJointedArmVisualizer armVisualizer = new DoubleJointedArmVisualizer(
		"",
		2.5,
		2.5,
		DoubleJointedArm.FIRST_JOINT_LENGTH_METERS,
		DoubleJointedArm.SECOND_JOINT_LENGTH_METERS
	);

	public static ArrayList<Translation2d> points;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		points = new ArrayList<>();

		// First segment: Move up along the Y-axis from (-0.3, 0.2) to (-0.3, 0.8)
		double startY = 0.2;
		double endY = 1;
		double x1 = -0.4;
		double step = 0.05;
		for (double y = startY; y <= endY; y += step) {
			points.add(new Translation2d(x1, y));
		}

		// Second segment: Move right along the X-axis from (-0.3, 0.8) to (0.3, 0.8)
		double x2 = 0.4;
		for (double x = x1 + step; x <= x2; x += step) {
			points.add(new Translation2d(x, endY));
		}

		// Third segment: Move down along the Y-axis from (0.3, 0.8) to (0.3, 0.2)
		for (double y = endY - step; y >= startY; y -= step) {
			points.add(new Translation2d(x2, y));
		}

		armVisualizer.showPath(points);
	}

	public void periodic() {
		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();

		armVisualizer.setAngles(arm.getFirstJointAngle(), arm.getSecondJointAngle());
		armVisualizer.setTargetPositionMeters(arm.getPositionMeters());

		CommandScheduler.getInstance().run(); // Should be last
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public DoubleJointedArm getArm() {
		return arm;
	}

}
