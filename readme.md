# PCGPIO

**A powerful and versatile solution for adding GPIO to a PC by for controlling microcontroller-based hardware controllers like Arduino AVR, ESP32, STM32, and Teensy via serial communication.**

PCGPIO simplifies interaction with your hardware, offering a robust and intuitive interface for hobbyists and professionals alike. Whether you're building a custom project or developing industrial automation solutions, PCGPIO empowers you to:

*   Read and write digital and analog pins with ease
*   Configure pin modes (input, output, input pullup) for flexible hardware control
*   Attach and detach interrupts to handle real-time events
*   Control PWM output for precise analog-like functionality
*   Manage timers for scheduled or periodic tasks
*   Perform efficient bulk read and write operations on multiple pins

Built with asynchronous operations, PCGPIO ensures your applications remain responsive and efficient. It also features auto-detection of serial ports, robust error handling, and advanced configuration options for power users.

## Features

*   **Asynchronous Operations:** All interactions with the controller are asynchronous, enabling concurrent task handling without blocking your application.
*   **Auto-Detection:** Automatically detects the serial port connected to your PCGPIO controller, streamlining setup.
*   **Error Handling:** Custom exceptions (`PCGPIOConnectionException` and `PCGPIOCommandException`) deliver clear, actionable error messages.
*   **Advanced Serial Configuration:** Customize baud rate, parity, stop bits, and data bits to match your hardware needs.
*   **Interrupt Support:** Attach interrupt handlers to pins with trigger modes (rising, falling, change) for real-time responsiveness.
*   **Timer Functionality:** Start and stop timers with microsecond precision for accurate timing control.
*   **Bulk Operations:** Read and write multiple digital pins in a single command, optimizing performance.
*   **Raw Command Support:** Send custom commands directly to the controller for advanced or proprietary operations.

## Contributing

We welcome contributions! Please submit issues and pull requests to help improve PCGPIO.

## License

This project is licensed under the GNU General Public License version 3 (GPLv3). See the [LICENSE](https://www.gnu.org/licenses/gpl-3.0.en.html) file for details.
- - -
# Library Usage
## Table of Contents

*   [API Documentation](#api-documentation)
    *   [Connection Management](#connection-management)
    *   [Pin Control](#pin-control)
    *   [Interrupts](#interrupts)
    *   [Timers](#timers)
    *   [Bulk Operations](#bulk-operations)
    *   [Advanced Features](#advanced-features)
    *   [Stepper Motor Control](#stepper-motor-control)
    *   [LED Strip Control](#led-strip-control)
*   [Examples](#examples)

## API Documentation

This section covers all public methods in the PCGPIO library, grouped by functionality. Each method includes its signature, description, parameters, return type, and possible exceptions.

### Connection Management

*   #### `public PCGPIO(string? portName = null)`
    
    Initializes a new instance of the PCGPIO class.
    
    **Parameters:**
    
    *   `portName` (string?, optional): The serial port name (e.g., "COM3"). If null, auto-detects the port.
    
    **Returns:** None (constructor)
    
    **Exceptions:** None
    
*   #### `public Task ConnectAsync(bool autoNegotiatePortSpeed = true)`
    
    Establishes a serial connection to the controller.
    
    **Parameters:**
    
    *   `autoNegotiatePortSpeed` (bool, optional): If true, negotiates the optimal baud rate (default: true).
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `PCGPIOConnectionException`: Thrown if the connection fails.
*   #### `public void Disconnect()`
    
    Closes the serial connection.
    
    **Parameters:** None
    
    **Returns:** None
    
    **Exceptions:** None
    
*   #### `public Task ResetControllerAsync()`
    
    Resets the controller via a software or hardware command.
    
    **Parameters:** None
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `PCGPIOConnectionException`: Thrown if not connected.

### Pin Control

*   #### `public Task ReadDigitalPinAsync(int pin)`
    
    Reads the state of a digital pin.
    
    **Parameters:**
    
    *   `pin` (int): The pin number.
    
    **Returns:** `bool` (true = HIGH, false = LOW)
    
    **Exceptions:**
    
    *   `ArgumentException`: If pin number is invalid.
    *   `PCGPIOCommandException`: If the command fails.
*   #### `public Task WriteDigitalPinAsync(int pin, bool value)`
    
    Writes a value to a digital pin.
    
    **Parameters:**
    
    *   `pin` (int): The pin number.
    *   `value` (bool): The value to write (true = HIGH, false = LOW).
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If pin number is invalid.
*   #### `public Task SetPinModeAsync(int pin, PinMode mode)`
    
    Sets the mode of a pin (e.g., Input, Output).
    
    **Parameters:**
    
    *   `pin` (int): The pin number.
    *   `mode` (PinMode): The pin mode (e.g., `PinMode.Input`, `PinMode.Output`).
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If pin or mode is invalid.
*   #### `public Task ReadAnalogPinAsync(int analogPin)`
    
    Reads the value of an analog pin.
    
    **Parameters:**
    
    *   `analogPin` (int): The analog pin number.
    
    **Returns:** `int` (typically 0-1023)
    
    **Exceptions:**
    
    *   `ArgumentException`: If pin number is invalid.
    *   `PCGPIOCommandException`: If the command fails.
*   #### `public Task WritePwmAsync(int pin, int value)`
    
    Sets the PWM duty cycle on a pin.
    
    **Parameters:**
    
    *   `pin` (int): The PWM-capable pin number.
    *   `value` (int): Duty cycle (0-255).
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If pin or value is invalid.

### Interrupts

*   #### `public Task AttachInterruptAsync(int pin, InterruptMode mode, Action callback)`
    
    Attaches an interrupt handler to a pin.
    
    **Parameters:**
    
    *   `pin` (int): The pin number.
    *   `mode` (InterruptMode): Trigger mode (e.g., `Rising`, `Falling`, `Change`).
    *   `callback` (Action): The method to call when the interrupt occurs.
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If pin or mode is invalid.
*   #### `public Task DetachInterruptAsync(int pin)`
    
    Detaches an interrupt from a pin.
    
    **Parameters:**
    
    *   `pin` (int): The pin number.
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If pin is invalid.

### Timers

*   #### `public Task StartTimerAsync(int timerId, uint microseconds, Action callback)`
    
    Starts a timer with microsecond precision.
    
    **Parameters:**
    
    *   `timerId` (int): The timer identifier.
    *   `microseconds` (uint): Interval in microseconds.
    *   `callback` (Action): The method to call when the timer triggers.
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If timerId or microseconds is invalid.
*   #### `public Task StopTimerAsync(int timerId)`
    
    Stops a running timer.
    
    **Parameters:**
    
    *   `timerId` (int): The timer identifier.
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If timerId is invalid.

### Bulk Operations

*   #### `public Task ReadDigitalPinsAsync(int[] pins)`
    
    Reads the states of multiple digital pins in one command.
    
    **Parameters:**
    
    *   `pins` (int\[\]): Array of pin numbers.
    
    **Returns:** `bool[]` (array of pin states)
    
    **Exceptions:**
    
    *   `ArgumentException`: If any pin is invalid.
    *   `PCGPIOCommandException`: If the command fails.
*   #### `public Task WriteDigitalPinsAsync(Dictionary pinValues)`
    
    Writes values to multiple digital pins in one command.
    
    **Parameters:**
    
    *   `pinValues` (Dictionary): Pin numbers and their values.
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If any pin is invalid.

### Advanced Features

*   #### `public Task SendRawCommandAsync(string command)`
    
    Sends a raw command to the controller and returns its response.
    
    **Parameters:**
    
    *   `command` (string): The raw command string.
    
    **Returns:** `string` (controller’s response)
    
    **Exceptions:**
    
    *   `ArgumentException`: If command is null or empty.
    *   `PCGPIOCommandException`: If the command fails.
*   #### `public Task SetSerialConfigAsync(int baudRate, Parity parity, int dataBits, StopBits stopBits)`
    
    Configures serial communication settings.
    
    **Parameters:**
    
    *   `baudRate` (int): Baud rate (e.g., 9600, 115200).
    *   `parity` (Parity): Parity setting (e.g., `Parity.None`).
    *   `dataBits` (int): Number of data bits (e.g., 8).
    *   `stopBits` (StopBits): Stop bits (e.g., `StopBits.One`).
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If parameters are invalid.

### Stepper Motor Control

*   #### `public Task StepMotorAsync(int motorId, int steps, bool direction)`
    
    Controls a stepper motor connected to the controller.
    
    **Parameters:**
    
    *   `motorId` (int): The motor identifier.
    *   `steps` (int): Number of steps to move.
    *   `direction` (bool): true = clockwise, false = counterclockwise.
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If motorId or steps is invalid.
*   #### `public Task SetMotorSpeedAsync(int motorId, int speed)`
    
    Sets the speed of a stepper motor.
    
    **Parameters:**
    
    *   `motorId` (int): The motor identifier.
    *   `speed` (int): Speed in steps per second.
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If motorId or speed is invalid.

### LED Strip Control

*   #### `public Task SetLedColorAsync(int ledIndex, byte r, byte g, byte b)`
    
    Sets the color of an LED in a strip (e.g., WS2812).
    
    **Parameters:**
    
    *   `ledIndex` (int): The LED index in the strip.
    *   `r` (byte): Red value (0-255).
    *   `g` (byte): Green value (0-255).
    *   `b` (byte): Blue value (0-255).
    
    **Returns:** `Task`
    
    **Exceptions:**
    
    *   `ArgumentException`: If ledIndex is invalid.
*   #### `public Task UpdateLedStripAsync()`
    
    Updates the LED strip with the set colors.
    
    **Parameters:** None
    
    **Returns:** `Task`
    
    **Exceptions:** None
    

## Examples

### Blinking an LED

```
using PCGPIOLib;

PCGPIO gpio = new PCGPIO();
await gpio.ConnectAsync();
await gpio.SetPinModeAsync(13, PinMode.Output);

while (true)
{
    await gpio.WriteDigitalPinAsync(13, true);  // LED on
    await Task.Delay(1000);
    await gpio.WriteDigitalPinAsync(13, false); // LED off
    await Task.Delay(1000);
}
```

### Using an Interrupt

```
using PCGPIOLib;

PCGPIO gpio = new PCGPIO();
await gpio.ConnectAsync();

await gpio.SetPinModeAsync(2, PinMode.Input);
await gpio.AttachInterruptAsync(2, InterruptMode.Rising, () =>
{
    Console.WriteLine("Button pressed!");
});
```

© 2025 ByronAP. All rights reserved.