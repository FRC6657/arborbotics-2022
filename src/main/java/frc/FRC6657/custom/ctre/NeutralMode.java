package frc.FRC6657.custom.ctre;

public enum NeutralMode {
    
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
	NeutralMode(int value)
	{
		this.value = value;
	}

}
