package frc.utils.digitalinput.chooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

public class ChooserDigitalInput implements IDigitalInput {

	private final SendableChooser<Boolean> isTrueSendableChooser;
	private boolean inverted = false;

	public ChooserDigitalInput(boolean inverted) {
		this.isTrueSendableChooser = new SendableChooser<>();
		isTrueSendableChooser.setDefaultOption(
			String.valueOf(ChooserDigitalInputConstants.DEFAULT_STATE),
			ChooserDigitalInputConstants.DEFAULT_STATE
		);
		isTrueSendableChooser.addOption("true", true);
		isTrueSendableChooser.addOption("false", false);

		SmartDashboard.putData(isTrueSendableChooser);

		this.inverted = inverted;
	}

	public ChooserDigitalInput() {
		this.isTrueSendableChooser = new SendableChooser<>();
		isTrueSendableChooser.setDefaultOption(
			String.valueOf(ChooserDigitalInputConstants.DEFAULT_STATE),
			ChooserDigitalInputConstants.DEFAULT_STATE
		);
		isTrueSendableChooser.addOption("true", true);
		isTrueSendableChooser.addOption("false", false);

		SmartDashboard.putData(ChooserDigitalInputConstants.CHOOSER_NAME, isTrueSendableChooser);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = (inverted) ? !isTrueSendableChooser.getSelected() : isTrueSendableChooser.getSelected();
		inputs.debouncedValue = inputs.nonDebouncedValue;
	}

}
