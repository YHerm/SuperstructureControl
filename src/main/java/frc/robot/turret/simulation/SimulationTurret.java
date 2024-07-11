package frc.robot.turret.simulation;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.turret.ITurret;
import frc.robot.turret.TurretInputsAutoLogged;

import static frc.robot.turret.simulation.SimulationConstants.GEARING;
import static frc.robot.turret.simulation.SimulationConstants.JKG_METER_SQ;
import static frc.robot.turret.simulation.SimulationConstants.MOTOR_CONFIGURATION;


public class SimulationTurret implements ITurret {

    private SimpleMotorSimulation motor;
    private VelocityDutyCycle velocityDutyCycle;
    private PositionDutyCycle positionDutyCycle;
    private VoltageOut voltageOut;

    public SimulationTurret() {
        motor = new SimpleMotorSimulation(
                DCMotor.getKrakenX60(SimulationConstants.AMOUNT_OF_MOTORS),
                GEARING,
                JKG_METER_SQ
        );

        motor.applyConfiguration(MOTOR_CONFIGURATION);

        positionDutyCycle = new PositionDutyCycle(0);
        velocityDutyCycle = new VelocityDutyCycle(0);
        voltageOut = new VoltageOut(0);
    }

    @Override
    public void setVelocity(double velocity) {
        motor.setControl(velocityDutyCycle.withVelocity(velocity));
    }

    @Override
    public void setPosition(Rotation2d angle) {
        motor.setControl(positionDutyCycle.withPosition(angle.getRotations()));
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setPower(double power) {
        motor.setPower(power);
    }

    @Override
    public void stop() {
        motor.stop();
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.position = motor.getPosition();
        inputs.velocity = motor.getVelocity();

        inputs.targetPosition = Rotation2d.fromRotations(velocityDutyCycle.Velocity);
        inputs.targetVelocity = Rotation2d.fromRotations(positionDutyCycle.Position);
    }

}
