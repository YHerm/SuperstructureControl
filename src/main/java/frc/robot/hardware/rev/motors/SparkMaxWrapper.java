package frc.robot.hardware.rev.motors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.phoenix6.Phoenix6Utils;
import frc.robot.hardware.rev.REVUtils;
import frc.robot.hardware.rev.SparkMaxConfiguration;
import frc.utils.Conversions;

public class SparkMaxWrapper extends SparkMax {

	private static final int DEFAULT_CONFIG_NUMBER_OF_TRIES = 1;

	public SparkMaxWrapper(SparkMaxDeviceID deviceID) {
		super(deviceID.id(), deviceID.type());

		// TODO super.configure()
//		super.restoreFactoryDefaults();
	}

	public double getVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}

	public Rotation2d getVelocityAnglePerSecond() {
		return Rotation2d.fromRotations(Conversions.perMinuteToPerSecond(getEncoder().getVelocity()));
	}

	public REVLibError applyConfiguration(SparkMaxConfiguration configuration, int numberOfTries) {
		return REVUtils.checkWithRetry(() -> super.configure(configuration.getSparkMaxConfig(), configuration.getResetMode(), configuration.getPersistMode()), numberOfTries);
	}

	public REVLibError applyConfiguration(SparkMaxConfiguration configuration) {
		return applyConfiguration(configuration, DEFAULT_CONFIG_NUMBER_OF_TRIES);
	}

}
