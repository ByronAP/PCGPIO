using System;

namespace PCGPIOLib
{
    /// <summary>
    /// Exception thrown for errors related to establishing or maintaining a connection with the controller.
    /// This includes serial port failures, timeouts, or communication issues.
    /// </summary>
    public class PCGPIOConnectionException : Exception
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="PCGPIOConnectionException"/> class with a specified error message.
        /// </summary>
        /// <param name="message">The message that describes the error.</param>
        public PCGPIOConnectionException(string message) : base(message) { }

        /// <summary>
        /// Initializes a new instance of the <see cref="PCGPIOConnectionException"/> class with a specified error message and inner exception.
        /// </summary>
        /// <param name="message">The message that describes the error.</param>
        /// <param name="innerException">The exception that caused this exception, or null if none.</param>
        public PCGPIOConnectionException(string message, Exception? innerException = null) : base(message, innerException) { }
    }
}