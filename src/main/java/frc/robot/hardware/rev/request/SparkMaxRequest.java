package frc.robot.hardware.rev.request;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;
import frc.utils.Conversions;

import java.util.function.Function;

public class SparkMaxRequest<T> implements IRequest<T> {

	private final CANSparkBase.ControlType controlType;
	private final int pidSlot;
	private final Function<T, Double> feedforwardCalculator;
	private final Function<T, Double> setPointToDoubleConverter;
	private T setPoint;

	SparkMaxRequest(
		T setPoint,
		CANSparkBase.ControlType controlType,
		int pidSlot,
		Function<T, Double> feedforwardCalculator,
		Function<T, Double> setPointToDoubleConverter
	) {
		this.setPoint = setPoint;
		this.controlType = controlType;
		this.pidSlot = pidSlot;
		this.feedforwardCalculator = feedforwardCalculator;
		this.setPointToDoubleConverter = setPointToDoubleConverter;
	}

	SparkMaxRequest(T setPoint, CANSparkBase.ControlType controlType, int pidSlot, Function<T, Double> setPointToDoubleConverter) {
		this(setPoint, controlType, pidSlot, CANSparkMAX -> 0.0, setPointToDoubleConverter);
	}

	@Override
	public SparkMaxRequest<T> withSetPoint(T setPoint) {
		if (controlType == CANSparkBase.ControlType.kVelocity) {
			this.setPoint = (T) Rotation2d.fromRotations(Conversions.perSecondToPerMinute(((Rotation2d) setPoint).getRotations()));
			return this;
		}
		this.setPoint = setPoint;
		return this;
	}

	@Override
	public T getSetPoint() {
		return setPoint;
	}

	public Double getSetPointAsDouble() {
		return setPointToDoubleConverter.apply(setPoint);
	}

	public CANSparkBase.ControlType getControlType() {
		return controlType;
	}

	public int getPidSlot() {
		return pidSlot;
	}

	public double getFeedforwardGain() {
		return feedforwardCalculator.apply(setPoint);
	}

}
