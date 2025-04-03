package frc.robot.structures.doublejointedarm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.SmartJoystick;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.alerts.Alert;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class DoubleJointedArm extends GBSubsystem {

	public static final double ELBOW_LENGTH_METERS = 0.8;
	public static final double WRIST_LENGTH_METERS = 0.5;
	public static final double TOTAL_LENGTH_METERS = ELBOW_LENGTH_METERS + WRIST_LENGTH_METERS;

	private Rotation2d elbowAngle = new Rotation2d();
	private Rotation2d wristAngle = new Rotation2d();

	public DoubleJointedArm(String logPath) {
		super(logPath);
	}

	@Override
	protected void subsystemPeriodic() {
		Logger.recordOutput(getLogPath() + "/ElbowAngleDeg", elbowAngle.getDegrees());
		Logger.recordOutput(getLogPath() + "/WristAngleDeg", wristAngle.getDegrees());
		Logger.recordOutput(getLogPath() + "/PositionMeters", getPositionMeters());
	}

	public Rotation2d getElbowAngle() {
		return elbowAngle;
	}

	public Rotation2d getWristAngle() {
		return wristAngle;
	}

	public void setElbowAngle(Rotation2d elbowAngle) {
		this.elbowAngle = elbowAngle;
	}

	public void setWristAngle(Rotation2d wristAngle) {
		this.wristAngle = wristAngle;
	}

	public void setAngles(Rotation2d elbowAngle, Rotation2d wristAngle) {
		setElbowAngle(elbowAngle);
		setWristAngle(wristAngle);
	}

	public Translation2d getPositionMeters() {
		return toPositionMeters(elbowAngle, wristAngle);
	}

	public void setPosition(Translation2d positionMeters, boolean isElbowLeft) {
		Optional<ArmAngles> targetAngles = toAngles(positionMeters, isElbowLeft);
		if (targetAngles.isEmpty()) {
			new Alert(Alert.AlertType.ERROR, getLogPath() + "unreachable position").report();
			return;
		}
		setAngles(targetAngles.get().elbowAngle(), targetAngles.get().wristAngle());
	}

	private static Optional<ArmAngles> toAngles(Translation2d positionMeters, boolean isElbowLeft) {
		return DoubleJointedArmKinematics.toAngles(positionMeters, ELBOW_LENGTH_METERS, WRIST_LENGTH_METERS, isElbowLeft);
	}

	private static Translation2d toPositionMeters(Rotation2d elbowAngle, Rotation2d wristAngle) {
		return DoubleJointedArmKinematics.toPositionMeters(new ArmAngles(elbowAngle, wristAngle), ELBOW_LENGTH_METERS, WRIST_LENGTH_METERS);
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


}
