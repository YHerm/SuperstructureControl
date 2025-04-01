package frc.robot.visualizers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class DoubleJointedArmVisualizer {

    private final Mechanism2d mechanism;
    private final MechanismRoot2d root;
    private final MechanismLigament2d firstJoint;
    private final MechanismLigament2d secondJoint;

    public DoubleJointedArmVisualizer(double xMeters, double yMeters, double l1Meters, double l2Meters) {
        this.mechanism = new Mechanism2d(xMeters, yMeters);
        this.root = mechanism.getRoot("Arm", xMeters / 2.0, 0);
        this.firstJoint = new MechanismLigament2d("first joint", l1Meters, 0);
        this.secondJoint = new MechanismLigament2d("first joint", l2Meters, 0, 10.0F, new Color8Bit(Color.kPurple));

        firstJoint.append(secondJoint);
        root.append(firstJoint);
        SmartDashboard.putData("Mech2d", mechanism);
    }

    public void setAngles(Rotation2d a1, Rotation2d a2) {
        firstJoint.setAngle(a1);
        secondJoint.setAngle(toFloorRelative(a1, a2));
    }

    private static Rotation2d toFloorRelative(Rotation2d floorRelativeAngle, Rotation2d jointRelativeAngle) {
        return jointRelativeAngle.minus(floorRelativeAngle);
    }

}
