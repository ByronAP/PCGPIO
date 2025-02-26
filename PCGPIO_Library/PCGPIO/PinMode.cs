namespace PCGPIOLib
{
    /// <summary>
    /// Defines the operational modes for digital pins on the controller.
    /// These modes determine whether a pin functions as an input or output and its electrical behavior.
    /// </summary>
    public enum PinMode
    {
        /// <summary>
        /// Pin is configured as an input to read digital signals.
        /// </summary>
        Input = 0,

        /// <summary>
        /// Pin is configured as an output to write digital signals.
        /// </summary>
        Output = 1,

        /// <summary>
        /// Pin is configured as an input with an internal pull-up resistor enabled.
        /// </summary>
        InputPullup = 2
    }
}