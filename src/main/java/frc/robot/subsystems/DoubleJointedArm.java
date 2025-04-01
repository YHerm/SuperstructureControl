package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.SmartJoystick;
import org.littletonrobotics.junction.Logger;

public class DoubleJointedArm extends GBSubsystem {

	public static final double FIRST_JOINT_LENGTH_METERS = 0.8;
	public static final double SECOND_JOINT_LENGTH_METERS = 0.5;

	public static final double TOTAL_ARM_LENGTH = FIRST_JOINT_LENGTH_METERS + SECOND_JOINT_LENGTH_METERS;
	public static final double FIRST_JOINT_PERCENT_FROM_ARM = FIRST_JOINT_LENGTH_METERS / TOTAL_ARM_LENGTH;
	public static final double SECOND_JOINT_PERCENT_FROM_ARM = SECOND_JOINT_LENGTH_METERS / TOTAL_ARM_LENGTH;

	private Rotation2d firstJointAngle = new Rotation2d();
	private Rotation2d secondJointAngle = new Rotation2d();

	public DoubleJointedArm(String logPath) {
		super(logPath);
	}

	@Override
	protected void subsystemPeriodic() {
		Logger.recordOutput(getLogPath() + "/FirstJointAngleDeg", firstJointAngle.getDegrees());
		Logger.recordOutput(getLogPath() + "/SecondJointAngleDeg", secondJointAngle.getDegrees());
		Logger.recordOutput(getLogPath() + "/PositionMeters", getPositionMeters());
	}

	public Rotation2d getFirstJointAngle() {
		return firstJointAngle;
	}

	public Rotation2d getSecondJointAngle() {
		return secondJointAngle;
	}

	public void setFirstJointAngle(Rotation2d firstJointAngle) {
		this.firstJointAngle = firstJointAngle;
	}

	public void setSecondJointAngle(Rotation2d secondJointAngle) {
		this.secondJointAngle = secondJointAngle;
	}

	public void setAngles(Rotation2d firstJointAngle, Rotation2d secondJointAngle) {
		setFirstJointAngle(firstJointAngle);
		setSecondJointAngle(secondJointAngle);
	}

	public Translation2d getPositionMeters() {
		return toPositionMeters(firstJointAngle, secondJointAngle);
	}

	public void setPosition(Translation2d positionMeters, boolean firstJointLeft) {
		double distanceMeters = positionMeters.getDistance(new Translation2d());

		double alphaRads = Math.atan2(positionMeters.getY(), positionMeters.getX());
		double omegaRads = Math.acos((distanceMeters * FIRST_JOINT_PERCENT_FROM_ARM) / FIRST_JOINT_LENGTH_METERS);

		if (!firstJointLeft) {
			omegaRads = -omegaRads;
		}

		setAngles(Rotation2d.fromRadians(alphaRads + omegaRads), Rotation2d.fromRadians(alphaRads - omegaRads));
	}

	public void applyTestBinds(SmartJoystick joystick) {
		joystick.A.onTrue(new InstantCommand(() -> setAngles(Rotation2d.fromDegrees(140), Rotation2d.fromDegrees(195))));
		joystick.B.onTrue(new InstantCommand(() -> setAngles(Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(-45))));
		joystick.X.onTrue(new InstantCommand(() -> setAngles(Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(75))));
		joystick.Y.onTrue(new InstantCommand(() -> setAngles(Rotation2d.fromDegrees(75), Rotation2d.fromDegrees(120))));

		joystick.POV_DOWN
			.onTrue(new InstantCommand(() -> setPosition(toPositionMeters(Rotation2d.fromDegrees(140), Rotation2d.fromDegrees(195)), false)));
		joystick.POV_RIGHT
			.onTrue(new InstantCommand(() -> setPosition(toPositionMeters(Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(-45)), true)));
		joystick.POV_LEFT
			.onTrue(new InstantCommand(() -> setPosition(toPositionMeters(Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(75)), true)));
		joystick.POV_UP
			.onTrue(new InstantCommand(() -> setPosition(toPositionMeters(Rotation2d.fromDegrees(75), Rotation2d.fromDegrees(120)), false)));
	}

	private static Translation2d toPositionMeters(Rotation2d firstJointAngle, Rotation2d secondJointAngle) {
		double xMeters = FIRST_JOINT_LENGTH_METERS * firstJointAngle.getCos() + SECOND_JOINT_LENGTH_METERS * secondJointAngle.getCos();
		double yMeters = FIRST_JOINT_LENGTH_METERS * firstJointAngle.getSin() + SECOND_JOINT_LENGTH_METERS * secondJointAngle.getSin();
		return new Translation2d(xMeters, yMeters);
	}

}
