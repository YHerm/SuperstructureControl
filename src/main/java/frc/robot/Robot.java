// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.structures.doublejointedarm.DoubleJointedArm;
import frc.robot.structures.doublejointedarm.DoubleJointedArmVisualizer;
import frc.utils.battery.BatteryUtil;

import java.util.List;

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
		DoubleJointedArm.ELBOW_LENGTH_METERS,
		DoubleJointedArm.WRIST_LENGTH_METERS
	);

	public static Trajectory pathUp;
	public static Trajectory pathLeft;
	public static Trajectory pathDown;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		pathUp = TrajectoryGenerator.generateTrajectory(
			new Pose2d(new Translation2d(-0.4, 0.05), Rotation2d.fromDegrees(90)),
			List.of(),
			new Pose2d(new Translation2d(-0.4, 1), Rotation2d.fromDegrees(90)),
			new TrajectoryConfig(2, 2).setStartVelocity(0).setEndVelocity(0)
		);

		pathLeft = TrajectoryGenerator.generateTrajectory(
			new Pose2d(new Translation2d(-0.4, 1), Rotation2d.fromDegrees(0)),
			List.of(),
			new Pose2d(new Translation2d(0.4, 1), Rotation2d.fromDegrees(0)),
			new TrajectoryConfig(2, 2).setStartVelocity(0).setEndVelocity(0)
		);
		pathDown = TrajectoryGenerator.generateTrajectory(
			new Pose2d(new Translation2d(0.4, 1), Rotation2d.fromDegrees(-90)),
			List.of(),
			new Pose2d(new Translation2d(0.4, 0.05), Rotation2d.fromDegrees(-90)),
			new TrajectoryConfig(2, 2).setStartVelocity(0).setEndVelocity(0)
		);
	}

	public void periodic() {
		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();

		armVisualizer.setAngles(arm.getElbowAngle(), arm.getWristAngle());
		armVisualizer.setTargetPositionMeters(arm.getPositionMeters());

		CommandScheduler.getInstance().run(); // Should be last
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public DoubleJointedArm getArm() {
		return arm;
	}

	public DoubleJointedArmVisualizer getArmVisualizer() {
		return armVisualizer;
	}

}
