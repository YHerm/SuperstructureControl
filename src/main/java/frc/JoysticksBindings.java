package frc;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.visualizers.FollowPathCommand;

public class JoysticksBindings {

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	public static void configureBindings(Robot robot) {
		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
//		robot.getArm().applyTestBinds(usedJoystick);
		usedJoystick.A.onTrue(
			new InstantCommand(() -> robot.getArmVisualizer().showPath(Robot.pathDown)).andThen(new WaitCommand(0.3))
				.andThen(new FollowPathCommand(Robot.pathDown, p -> robot.getArm().setPosition(p, false)))
		);
		usedJoystick.B.onTrue(
				new InstantCommand(() -> robot.getArmVisualizer().showPath(Robot.pathLeft)).andThen(new WaitCommand(0.3))
						.andThen(new FollowPathCommand(Robot.pathLeft, p -> robot.getArm().setPosition(p, false)))
		);
		usedJoystick.Y.onTrue(
				new InstantCommand(() -> robot.getArmVisualizer().showPath(Robot.pathUp)).andThen(new WaitCommand(0.3))
						.andThen(new FollowPathCommand(Robot.pathUp, p -> robot.getArm().setPosition(p, false)))
		);
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...
	}

	private static void fifthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SIXTH_JOYSTICK;
		// bindings...
	}

}
