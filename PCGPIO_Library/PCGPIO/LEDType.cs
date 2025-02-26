namespace PCGPIOLib
{
    /// <summary>
    /// Defines the types of LED strips supported by the controller.
    /// These types correspond to different LED strip protocols or chipsets.
    /// </summary>
    public enum LEDType
    {
        /// <summary>
        /// WS2812 LED strip type.
        /// This is a common type of addressable LED strip using a single data line.
        /// </summary>
        WS2812 = 0,

        /// <summary>
        /// APA102 LED strip type.
        /// Another type of addressable LED strip using both data and clock lines for communication.
        /// </summary>
        APA102 = 1
    }
}