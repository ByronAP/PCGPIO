using System;

namespace PCGPIOLib
{
    /// <summary>
    /// Exception thrown when an error occurs during command execution on the controller.
    /// This may result from invalid commands, parameters, or unexpected controller responses.
    /// </summary>
    public class PCGPIOCommandException : Exception
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="PCGPIOCommandException"/> class with a specified error message.
        /// </summary>
        /// <param name="message">The message that describes the error.</param>
        public PCGPIOCommandException(string message) : base(message) { }
    }
}