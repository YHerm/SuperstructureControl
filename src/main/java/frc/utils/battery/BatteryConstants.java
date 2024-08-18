package frc.utils.battery;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.utils.alerts.Alert;

class BatteryConstants {

	protected static final String LOG_PATH = "Battery/";
	protected static final String ALERT_LOG_PATH = Alert.ALERT_LOG_PATH + LOG_PATH;

	protected static final int POWER_DISTRIBUTION_CAN_ID = 20;
	protected static final PowerDistribution.ModuleType POWER_DISTRIBUTION_TYPE = PowerDistribution.ModuleType.kRev;

}
