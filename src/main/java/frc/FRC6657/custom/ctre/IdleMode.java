package frc.FRC6657.custom.ctre;

public enum IdleMode {
    
    Coast(0),
    Brake(1),
    HalfBrake(2);
    
    /**
	 * Value of NeutralMode
	 */
	public int value;
	/**
	 * Create NeutralMode from specified value
	 * @param value Value of NeutralMode
	 */
	IdleMode(int value)
	{
		this.value = value;
	}

}
