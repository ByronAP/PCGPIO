namespace PCGPIOLib
{
    /// <summary>
    /// Defines the trigger modes for interrupts on the controller.
    /// These modes specify the conditions under which an interrupt is activated based on pin state changes.
    /// </summary>
    public enum InterruptMode
    {
        /// <summary>
        /// Interrupt triggers on a rising edge (transition from low to high).
        /// </summary>
        Rising = 2,

        /// <summary>
        /// Interrupt triggers on a falling edge (transition from high to low).
        /// </summary>
        Falling = 3,

        /// <summary>
        /// Interrupt triggers on any change (either rising or falling edge).
        /// </summary>
        Change = 1
    }
}