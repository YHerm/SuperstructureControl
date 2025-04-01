package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.SmartJoystick;
import org.littletonrobotics.junction.Logger;

public class DoubleJointedArm extends GBSubsystem {

	public static final double FIRST_JOINT_LENGTH_METERS = 0.8;
	public static final double SECOND_JOINT_LENGTH_METERS = 0.5;

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
		double xMeters = FIRST_JOINT_LENGTH_METERS * firstJointAngle.getCos() + SECOND_JOINT_LENGTH_METERS * secondJointAngle.getCos();
		double yMeters = FIRST_JOINT_LENGTH_METERS * firstJointAngle.getSin() + SECOND_JOINT_LENGTH_METERS * secondJointAngle.getSin();
		return new Translation2d(xMeters, yMeters);
	}

	public void applyTestBinds(SmartJoystick joystick) {
		joystick.A.onTrue(new InstantCommand(() -> setAngles(Rotation2d.fromDegrees(100), Rotation2d.fromDegrees(210))));
		joystick.B.onTrue(new InstantCommand(() -> setAngles(Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(80))));
		joystick.X.onTrue(new InstantCommand(() -> setAngles(Rotation2d.fromDegrees(150), Rotation2d.fromDegrees(75))));
		joystick.Y.onTrue(new InstantCommand(() -> setAngles(Rotation2d.fromDegrees(95), Rotation2d.fromDegrees(85))));
	}

}
