package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.geometry.Rotation2d;

public class Phoenix6RequestBuilder {

	public static phoenix6FeedForwardRequest build(PositionVoltage positionVoltage, double defaultArbitraryFeedForward, boolean enableFOC) {
		return new phoenix6FeedForwardRequest(
			Rotation2d.fromRotations(positionVoltage.Position),
			positionVoltage.withEnableFOC(enableFOC),
			setPoint -> positionVoltage.withPosition(setPoint.getRotations()),
			positionVoltage::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static phoenix6FeedForwardRequest build(VelocityVoltage velocityVoltage, double defaultArbitraryFeedForward, boolean enableFOC) {
		return new phoenix6FeedForwardRequest(
			Rotation2d.fromRotations(velocityVoltage.Velocity),
			velocityVoltage.withEnableFOC(enableFOC),
			setPoint -> velocityVoltage.withVelocity(setPoint.getRotations()),
			velocityVoltage::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static phoenix6FeedForwardRequest build(
		MotionMagicVoltage motionMagicVoltage,
		double defaultArbitraryFeedForward,
		boolean enableFOC
	) {
		return new phoenix6FeedForwardRequest(
			Rotation2d.fromRotations(motionMagicVoltage.Position),
			motionMagicVoltage.withEnableFOC(enableFOC),
			setPoint -> motionMagicVoltage.withPosition(setPoint.getRotations()),
			motionMagicVoltage::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static phoenix6FeedForwardRequest build(
		DynamicMotionMagicVoltage dynamicMotionMagicVoltage,
		double defaultArbitraryFeedForward,
		boolean enableFOC
	) {
		return new phoenix6FeedForwardRequest(
			Rotation2d.fromRotations(dynamicMotionMagicVoltage.Position),
			dynamicMotionMagicVoltage.withEnableFOC(enableFOC),
			setPoint -> dynamicMotionMagicVoltage.withPosition(setPoint.getRotations()),
			dynamicMotionMagicVoltage::withFeedForward,
			defaultArbitraryFeedForward
		);
	}

	public static Phoenix6Request<Double> build(VoltageOut voltageOut, boolean enableFOC) {
		return new Phoenix6Request<>(voltageOut.Output, voltageOut.withEnableFOC(enableFOC), voltageOut::withOutput);
	}

	public static Phoenix6Request<Double> build(TorqueCurrentFOC torqueCurrentFOC) {
		return new Phoenix6Request<>(torqueCurrentFOC.Output, torqueCurrentFOC, torqueCurrentFOC::withOutput);
	}

}
