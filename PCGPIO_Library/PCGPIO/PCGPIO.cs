using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Threading.Tasks;

namespace PCGPIOLib
{
    /// <summary>
    /// Provides an interface to communicate with and control a hardware controller
    /// via a serial connection. This class manages the connection, sends commands,
    /// and handles responses from the controller, including interrupts and timers.
    /// </summary>
    public class PCGPIO : IDisposable
    {
        private readonly SerialPort _serialPort;
        private readonly ConcurrentDictionary<string, TaskCompletionSource<string>> _commandResponses = new();
        private readonly ConcurrentDictionary<int, Action> _interruptCallbacks = new();
        private Task _readTask;
        private bool _isRunning;
        private readonly object _lock = new();

        // Pin and board configuration fields
        private string _boardType = "Unknown";
        private string _architecture = "Unknown";
        private uint _clockSpeedMhz = 0;
        private int _totalDigitalPins = -1;
        private int _totalAnalogPins = -1;
        private List<int> _pwmPins = new();
        private List<int> _interruptPins = new();
        private int _totalIntSlots = -1;
        private int _slotsInUse = -1;
        private bool[] _interruptSlotsInUse;
        private readonly Dictionary<int, int> _pinToSlot = new();
        private int _timerCount = 0;
        private int _freeRamBytes = 0;
        private uint _flashSizeBytes = 0;
        private string _chipId = "0";

        /// <summary>
        /// Event triggered when an interrupt occurs on a pin.
        /// </summary>
        public event Action<int> OnInterrupt;

        /// <summary>
        /// Event triggered when a timer fires on the controller.
        /// </summary>
        public event Action<int> OnTimer;

        /// <summary>
        /// Initializes a new instance of the <see cref="PCGPIO"/> class.
        /// </summary>
        /// <param name="portName">The name of the serial port to use. If null, the port is auto-detected.</param>
        public PCGPIO(string? portName = null)
        {
            portName ??= AutoDetectPort();
            _serialPort = new SerialPort(portName, 115200)
            {
                Parity = Parity.None,
                StopBits = StopBits.One,
                DataBits = 8,
                DtrEnable = false
            };
            OnInterrupt += HandleInterrupt;
            _readTask = Task.CompletedTask;
            OnTimer += (timerNum) => { };
            _interruptSlotsInUse = Array.Empty<bool>();
        }

        /// <summary>
        /// Asynchronously establishes a connection to the controller via the serial port.
        /// </summary>
        /// <param name="autoNegotiatePortSpeed">If true, negotiates the optimal baud rate with the controller.</param>
        /// <exception cref="PCGPIOConnectionException">Thrown if the connection attempt fails.</exception>
        public async Task ConnectAsync(bool autoNegotiatePortSpeed = true)
        {
            if (!_serialPort.IsOpen)
            {
                try
                {
                    _serialPort.Open();
                    _isRunning = true;
                    _readTask = Task.Run(ReadSerialDataAsync);
                    await PingAsync();
                    if (autoNegotiatePortSpeed)
                    {
                        await NegotiateBaudRateAsync();
                    }
                    await GetControllerInfoAsync();
                }
                catch (Exception ex)
                {
                    throw new PCGPIOConnectionException("Failed to connect to controller.", ex);
                }
            }
        }

        /// <summary>
        /// Disconnects from the controller by closing the serial port.
        /// </summary>
        public void Disconnect()
        {
            if (_serialPort.IsOpen)
            {
                _isRunning = false;
                _serialPort.Close();
            }
        }

        /// <summary>
        /// Asynchronously resets the controller using either a software command or hardware reset.
        /// </summary>
        /// <exception cref="PCGPIOConnectionException">Thrown if the reset or reconnection fails.</exception>
        public async Task ResetControllerAsync()
        {
            bool softwareResetSuccessful = false;
            string resetCommandId = Guid.NewGuid().ToString();

            lock (_lock)
            {
                if (!_serialPort.IsOpen)
                {
                    throw new PCGPIOConnectionException("Serial port is not open. Cannot reset controller.");
                }

                try
                {
                    _isRunning = false;
                    string command = $"Q,{resetCommandId}\n";
                    _serialPort.Write(command);
                    var tcs = new TaskCompletionSource<string>();
                    _commandResponses.TryAdd(resetCommandId, tcs);
                }
                catch (Exception ex)
                {
                    throw new PCGPIOConnectionException("Failed to send reset command.", ex);
                }
            }

            try
            {
                var tcs = _commandResponses[resetCommandId];
                var timeoutTask = Task.Delay(1000);
                var completedTask = await Task.WhenAny(tcs.Task, timeoutTask);

                if (completedTask != timeoutTask)
                {
                    softwareResetSuccessful = true;
                }
                _commandResponses.TryRemove(resetCommandId, out _);
            }
            catch (Exception)
            {
                // Ignore exceptions here; proceed to hardware reset if needed
            }
            finally
            {
                lock (_lock)
                {
                    if (_serialPort.IsOpen)
                    {
                        _serialPort.Close();
                    }
                }
            }

            if (!softwareResetSuccessful)
            {
                lock (_lock)
                {
                    try
                    {
                        _serialPort.DtrEnable = true;
                        Task.Delay(100).Wait();
                        _serialPort.DtrEnable = false;
                        if (_serialPort.IsOpen)
                        {
                            _serialPort.Close();
                        }
                    }
                    catch (Exception ex)
                    {
                        throw new PCGPIOConnectionException("Hardware reset via DTR failed.", ex);
                    }
                }
            }

            await Task.Delay(2000);

            lock (_lock)
            {
                try
                {
                    _serialPort.Open();
                    _isRunning = true;
                    _readTask = Task.Run(ReadSerialDataAsync);
                }
                catch (Exception ex)
                {
                    throw new PCGPIOConnectionException("Failed to reconnect after reset.", ex);
                }
            }

            await PingAsync();
            await GetControllerInfoAsync();
        }

        /// <summary>
        /// Retrieves detailed information about the controller, such as board type, pin configurations, and memory.
        /// </summary>
        /// <returns>A tuple containing the controller's detailed information.</returns>
        /// <exception cref="PCGPIOCommandException">Thrown if the response from the controller is invalid.</exception>
        public async Task<(string BoardType, string Architecture, uint ClockSpeedMhz, int DigitalPins, int AnalogPins, List<int> PwmPins, List<int> InterruptPins, int TotalIntSlots, int SlotsInUse, int TimerCount, int FreeRamBytes, uint FlashSizeBytes, string ChipId)> GetControllerInfoAsync()
        {
            string commandId = Guid.NewGuid().ToString();
            string command = $"G,{commandId}\n";
            string response = await SendCommandAsync(command, commandId);
            var parts = response.Split(',');
            if (parts.Length < 15 || parts[1] != "G")
            {
                throw new PCGPIOCommandException("Invalid controller info response.");
            }

            int index = 2;
            _boardType = parts[index++];
            _architecture = parts[index++];
            _clockSpeedMhz = uint.Parse(parts[index++]);
            _totalDigitalPins = int.Parse(parts[index++]);
            _totalAnalogPins = int.Parse(parts[index++]);
            int pwmCount = int.Parse(parts[index++]);
            _pwmPins = new List<int>();
            for (int i = 0; i < pwmCount; i++)
            {
                _pwmPins.Add(int.Parse(parts[index++]));
            }
            int interruptCount = int.Parse(parts[index++]);
            _interruptPins = new List<int>();
            for (int i = 0; i < interruptCount; i++)
            {
                _interruptPins.Add(int.Parse(parts[index++]));
            }
            _totalIntSlots = int.Parse(parts[index++]);
            _interruptSlotsInUse = new bool[_totalIntSlots];
            for (int i = 0; i < _totalIntSlots; i++)
            {
                _interruptSlotsInUse[i] = false;
            }
            _slotsInUse = int.Parse(parts[index++]);
            _timerCount = int.Parse(parts[index++]);
            _freeRamBytes = int.Parse(parts[index++]);
            _flashSizeBytes = uint.Parse(parts[index++]);
            _chipId = parts[index++];

            return (_boardType, _architecture, _clockSpeedMhz, _totalDigitalPins, _totalAnalogPins, _pwmPins, _interruptPins, _totalIntSlots, _slotsInUse, _timerCount, _freeRamBytes, _flashSizeBytes, _chipId);
        }

        /// <summary>
        /// Asynchronously reads the state of a digital pin on the controller.
        /// </summary>
        /// <param name="pin">The digital pin number to read.</param>
        /// <returns>True if the pin is HIGH, false if LOW.</returns>
        /// <exception cref="ArgumentException">Thrown if the pin number is invalid.</exception>
        /// <exception cref="PCGPIOCommandException">Thrown if the response is invalid.</exception>
        public async Task<bool> ReadDigitalPinAsync(int pin)
        {
            ValidatePin(pin);
            string commandId = Guid.NewGuid().ToString();
            string command = $"D,{pin},{commandId}\n";
            string response = await SendCommandAsync(command, commandId);
            return ParseDigitalReadResponse(response, pin);
        }

        /// <summary>
        /// Asynchronously sets the state of a digital pin on the controller.
        /// </summary>
        /// <param name="pin">The digital pin number to write to.</param>
        /// <param name="value">The value to set (true for HIGH, false for LOW).</param>
        /// <exception cref="ArgumentException">Thrown if the pin number is invalid.</exception>
        public async Task WriteDigitalPinAsync(int pin, bool value)
        {
            ValidatePin(pin);
            string commandId = Guid.NewGuid().ToString();
            string command = $"W,{pin},{(value ? 1 : 0)},{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        /// <summary>
        /// Asynchronously configures the mode of a digital pin on the controller.
        /// </summary>
        /// <param name="pin">The digital pin number to configure.</param>
        /// <param name="mode">The mode to set (e.g., Input, Output, InputPullup).</param>
        /// <exception cref="ArgumentException">Thrown if the pin number is invalid.</exception>
        public async Task SetPinModeAsync(int pin, PinMode mode)
        {
            ValidatePin(pin);
            string commandId = Guid.NewGuid().ToString();
            string command = $"M,{pin},{(int)mode},{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        /// <summary>
        /// Asynchronously attaches an interrupt to a pin on the controller.
        /// </summary>
        /// <param name="pin">The pin number to attach the interrupt to.</param>
        /// <param name="mode">The interrupt trigger mode (e.g., Rising, Falling, Change).</param>
        /// <param name="callback">The action to invoke when the interrupt occurs.</param>
        /// <exception cref="InvalidOperationException">Thrown if no interrupt slots are available or controller info is not retrieved.</exception>
        /// <exception cref="ArgumentException">Thrown if the pin does not support interrupts or the mode is invalid.</exception>
        public async Task AttachInterruptAsync(int pin, InterruptMode mode, Action callback)
        {
            if (_totalIntSlots == -1)
            {
                throw new InvalidOperationException("Controller info not retrieved. Call ConnectAsync first.");
            }
            ValidatePin(pin);
            if (!_interruptPins.Contains(pin))
            {
                throw new ArgumentException("Pin does not support interrupts.");
            }

            int slot = -1;
            for (int i = 0; i < _totalIntSlots; i++)
            {
                if (!_interruptSlotsInUse[i])
                {
                    slot = i;
                    _interruptSlotsInUse[i] = true;
                    break;
                }
            }
            if (slot == -1)
            {
                throw new InvalidOperationException("No interrupt slots available.");
            }

            int modeInt = mode switch
            {
                InterruptMode.Rising => 2,
                InterruptMode.Falling => 3,
                InterruptMode.Change => 1,
                _ => throw new ArgumentException("Invalid interrupt mode.")
            };

            string commandId = Guid.NewGuid().ToString();
            string command = $"I,{slot},{pin},{modeInt},{commandId}\n";
            await SendCommandAsync(command, commandId);

            _interruptCallbacks[pin] = callback;
            _pinToSlot[pin] = slot;
        }

        /// <summary>
        /// Asynchronously detaches an interrupt from a pin on the controller.
        /// </summary>
        /// <param name="pin">The pin number to detach the interrupt from.</param>
        /// <exception cref="ArgumentException">Thrown if the pin number is invalid.</exception>
        public async Task DetachInterruptAsync(int pin)
        {
            ValidatePin(pin);
            if (_pinToSlot.TryGetValue(pin, out int slot))
            {
                _interruptSlotsInUse[slot] = false;
                _pinToSlot.Remove(pin);
            }
            string commandId = Guid.NewGuid().ToString();
            string command = $"R,{pin},{commandId}\n";
            await SendCommandAsync(command, commandId);
            _interruptCallbacks.TryRemove(pin, out _);
        }

        /// <summary>
        /// Asynchronously reads the value of an analog pin on the controller.
        /// </summary>
        /// <param name="analogPin">The analog pin number to read.</param>
        /// <returns>The analog value (typically 0-1023 or controller-specific range).</returns>
        /// <exception cref="ArgumentException">Thrown if the analog pin number is invalid.</exception>
        /// <exception cref="PCGPIOCommandException">Thrown if the response is invalid.</exception>
        public async Task<int> ReadAnalogPinAsync(int analogPin)
        {
            ValidateAnalogPin(analogPin);
            string commandId = Guid.NewGuid().ToString();
            string command = $"A,{analogPin},{commandId}\n";
            string response = await SendCommandAsync(command, commandId);
            return ParseAnalogReadResponse(response, analogPin);
        }

        /// <summary>
        /// Asynchronously sets the PWM value for a pin on the controller.
        /// </summary>
        /// <param name="pin">The pin number to set the PWM value for.</param>
        /// <param name="value">The PWM duty cycle value (0-255).</param>
        /// <exception cref="ArgumentException">Thrown if the pin does not support PWM or the value is out of range.</exception>
        public async Task WritePwmAsync(int pin, int value)
        {
            ValidatePin(pin);
            if (!_pwmPins.Contains(pin))
            {
                throw new ArgumentException("Pin does not support PWM.");
            }
            if (value < 0 || value > 255) throw new ArgumentException("PWM value must be 0-255.");
            string commandId = Guid.NewGuid().ToString();
            string command = $"P,{pin},{value},{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        /// <summary>
        /// Asynchronously starts a timer on the controller with a specified interval.
        /// </summary>
        /// <param name="timerNum">The timer number to start (1-based index).</param>
        /// <param name="intervalUs">The interval in microseconds.</param>
        /// <exception cref="ArgumentException">Thrown if the timer number is invalid.</exception>
        public async Task StartTimerAsync(int timerNum, uint intervalUs)
        {
            ValidateTimer(timerNum);
            string commandId = Guid.NewGuid().ToString();
            string command = $"S,{timerNum},start,{intervalUs},{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        /// <summary>
        /// Asynchronously stops a timer on the controller.
        /// </summary>
        /// <param name="timerNum">The timer number to stop (1-based index).</param>
        /// <exception cref="ArgumentException">Thrown if the timer number is invalid.</exception>
        public async Task StopTimerAsync(int timerNum)
        {
            ValidateTimer(timerNum);
            string commandId = Guid.NewGuid().ToString();
            string command = $"S,{timerNum},stop,{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        /// <summary>
        /// Asynchronously reads the states of multiple digital pins on the controller.
        /// </summary>
        /// <param name="pins">An array of digital pin numbers to read.</param>
        /// <returns>A dictionary mapping pin numbers to their states (true for HIGH, false for LOW).</returns>
        /// <exception cref="ArgumentException">Thrown if any pin number is invalid.</exception>
        /// <exception cref="PCGPIOCommandException">Thrown if the response is invalid.</exception>
        public async Task<Dictionary<int, bool>> ReadDigitalPinsAsync(int[] pins)
        {
            foreach (var pin in pins) ValidatePin(pin);
            string commandId = Guid.NewGuid().ToString();
            string command = $"B,{string.Join(",", pins)},{commandId}\n";
            string response = await SendCommandAsync(command, commandId);
            return ParseBulkReadResponse(response, pins);
        }

        /// <summary>
        /// Asynchronously sets the states of multiple digital pins on the controller.
        /// </summary>
        /// <param name="pinValues">A dictionary mapping pin numbers to their desired states (true for HIGH, false for LOW).</param>
        /// <exception cref="ArgumentException">Thrown if any pin number is invalid.</exception>
        public async Task WriteDigitalPinsAsync(Dictionary<int, bool> pinValues)
        {
            foreach (var pin in pinValues.Keys) ValidatePin(pin);
            string values = string.Join(",", pinValues.Select(kv => $"{kv.Key}:{(kv.Value ? 1 : 0)}"));
            string commandId = Guid.NewGuid().ToString();
            string command = $"V,{values},{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        /// <summary>
        /// Asynchronously sends a raw command string to the controller and retrieves the response.
        /// </summary>
        /// <param name="command">The raw command string to send.</param>
        /// <returns>The response string from the controller.</returns>
        /// <exception cref="PCGPIOConnectionException">Thrown if the serial port is not open.</exception>
        /// <exception cref="TimeoutException">Thrown if the command times out.</exception>
        public async Task<string> SendRawCommandAsync(string command)
        {
            string commandId = Guid.NewGuid().ToString();
            string fullCommand = $"{command},{commandId}\n";
            return await SendCommandAsync(fullCommand, commandId);
        }

        /// <summary>
        /// Configures the serial port settings. Must be called before connecting.
        /// </summary>
        /// <param name="baudRate">The baud rate for serial communication.</param>
        /// <param name="parity">The parity setting for the serial port.</param>
        /// <param name="stopBits">The stop bits setting for the serial port.</param>
        /// <param name="dataBits">The data bits setting for the serial port.</param>
        /// <exception cref="InvalidOperationException">Thrown if the serial port is already open.</exception>
        public void ConfigureSerial(int baudRate = 1000000, Parity parity = Parity.None, StopBits stopBits = StopBits.One, int dataBits = 8)
        {
            if (_serialPort.IsOpen) throw new InvalidOperationException("Cannot configure serial port while connected.");
            _serialPort.BaudRate = baudRate;
            _serialPort.Parity = parity;
            _serialPort.StopBits = stopBits;
            _serialPort.DataBits = dataBits;
        }

        /// <summary>
        /// Retrieves a list of available serial ports on the system.
        /// </summary>
        /// <returns>An enumerable collection of serial port names.</returns>
        public IEnumerable<string> GetAvailablePorts()
        {
            return SerialPort.GetPortNames();
        }

        /// <summary>
        /// Asynchronously retrieves the current timestamp from the controller.
        /// </summary>
        /// <returns>The timestamp in microseconds since the controller started.</returns>
        /// <exception cref="PCGPIOCommandException">Thrown if the response is invalid.</exception>
        public async Task<ulong> GetTimestampAsync()
        {
            string commandId = Guid.NewGuid().ToString();
            string command = $"T,{commandId}\n";
            string response = await SendCommandAsync(command, commandId);
            var parts = response.Split(',');
            if (parts.Length == 3 && parts[1] == "T")
            {
                return ulong.Parse(parts[0]);
            }
            throw new PCGPIOCommandException("Invalid timestamp response.");
        }

        /// <summary>
        /// Asynchronously configures a stepper motor on the controller using specified pins.
        /// </summary>
        /// <param name="stepPin">The pin number for stepping the motor.</param>
        /// <param name="dirPin">The pin number for setting the motor direction.</param>
        /// <returns>The index assigned to the stepper motor by the controller.</returns>
        /// <exception cref="ArgumentException">Thrown if the pin numbers are invalid.</exception>
        /// <exception cref="PCGPIOCommandException">Thrown if the setup fails.</exception>
        public async Task<int> SetupStepperAsync(int stepPin, int dirPin)
        {
            ValidatePin(stepPin);
            ValidatePin(dirPin);
            string commandId = Guid.NewGuid().ToString();
            string command = $"N,setup,{stepPin},{dirPin},{commandId}\n";
            string response = await SendCommandAsync(command, commandId);
            if (response.Contains("Stepper"))
            {
                var parts = response.Split(' ');
                return int.Parse(parts[1]);
            }
            throw new PCGPIOCommandException("Failed to setup stepper.");
        }

        /// <summary>
        /// Asynchronously moves a stepper motor by a specified number of steps at a given speed.
        /// </summary>
        /// <param name="stepperIndex">The index of the stepper motor to move.</param>
        /// <param name="steps">The number of steps to move (positive for forward, negative for reverse).</param>
        /// <param name="stepsPerSecond">The speed in steps per second.</param>
        /// <exception cref="ArgumentException">Thrown if the stepper index is invalid.</exception>
        public async Task StepStepperAsync(int stepperIndex, long steps, uint stepsPerSecond)
        {
            if (stepperIndex < 0) throw new ArgumentException("Stepper index must be non-negative.");
            string commandId = Guid.NewGuid().ToString();
            string command = $"N,step,{stepperIndex},{steps},{stepsPerSecond},{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        /// <summary>
        /// Asynchronously releases resources for a stepper motor on the controller.
        /// </summary>
        /// <param name="stepperIndex">The index of the stepper motor to dispose.</param>
        /// <exception cref="ArgumentException">Thrown if the stepper index is invalid.</exception>
        public async Task DisposeStepperAsync(int stepperIndex)
        {
            if (stepperIndex < 0) throw new ArgumentException("Stepper index must be non-negative.");
            string commandId = Guid.NewGuid().ToString();
            string command = $"N,dispose,{stepperIndex},{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        /// <summary>
        /// Asynchronously configures an LED strip on the controller.
        /// </summary>
        /// <param name="type">The type of LED strip (e.g., WS2812, APA102).</param>
        /// <param name="dataPin">The pin number for the data line.</param>
        /// <param name="clockPin">The pin number for the clock line (use -1 if not applicable).</param>
        /// <param name="numPixels">The number of pixels in the LED strip.</param>
        /// <exception cref="ArgumentException">Thrown if the pin numbers are invalid.</exception>
        public async Task SetupLedStripAsync(LEDType type, int dataPin, int clockPin, int numPixels)
        {
            ValidatePin(dataPin);
            if (clockPin != -1) ValidatePin(clockPin);
            string commandId = Guid.NewGuid().ToString();
            string command = $"L,setup,{(int)type},{dataPin},{clockPin},{numPixels},{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        /// <summary>
        /// Asynchronously sets the color of a specific pixel in the LED strip.
        /// </summary>
        /// <param name="pixelIndex">The index of the pixel to set (0-based).</param>
        /// <param name="r">The red component value (0-255).</param>
        /// <param name="g">The green component value (0-255).</param>
        /// <param name="b">The blue component value (0-255).</param>
        /// <exception cref="ArgumentException">Thrown if the pixel index or RGB values are invalid.</exception>
        public async Task SetLedPixelAsync(int pixelIndex, int r, int g, int b)
        {
            if (pixelIndex < 0) throw new ArgumentException("Pixel index must be non-negative.");
            if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255)
                throw new ArgumentException("RGB values must be 0-255.");
            string commandId = Guid.NewGuid().ToString();
            string command = $"L,set,{pixelIndex},{r},{g},{b},{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        /// <summary>
        /// Asynchronously updates the LED strip to display the set pixel colors.
        /// </summary>
        public async Task ShowLedStripAsync()
        {
            string commandId = Guid.NewGuid().ToString();
            string command = $"L,show,{commandId}\n";
            await SendCommandAsync(command, commandId);
        }

        // Sends a command to the controller and awaits the response with a matching command ID.
        private async Task<string> SendCommandAsync(string command, string commandId)
        {
            lock (_lock)
            {
                if (!_serialPort.IsOpen) throw new PCGPIOConnectionException("Serial port is not open.");
                _serialPort.Write(command);
            }

            var tcs = new TaskCompletionSource<string>();
            _commandResponses.TryAdd(commandId, tcs);

            var timeoutTask = Task.Delay(5000);
            var completedTask = await Task.WhenAny(tcs.Task, timeoutTask);

            if (completedTask == timeoutTask)
            {
                _commandResponses.TryRemove(commandId, out _);
                throw new TimeoutException("Command timed out.");
            }

            _commandResponses.TryRemove(commandId, out _);
            return await tcs.Task;
        }

        // Continuously reads data from the serial port and processes responses.
        private async Task ReadSerialDataAsync()
        {
            while (_isRunning && _serialPort.IsOpen)
            {
                try
                {
                    string line = await Task.Run(() => _serialPort.ReadLine());
                    HandleResponse(line);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Serial read error: {ex.Message}");
                }
            }
        }

        // Processes incoming responses, routing them to command completions or event handlers.
        private void HandleResponse(string line)
        {
            var parts = line.Split(',');
            if (parts.Length < 3) return;

            string type = parts[1];
            string commandId = parts[^1]; // Last element is the ID

            if (type == "I" && commandId == "INT")
            {
                int pin = int.Parse(parts[2]);
                OnInterrupt?.Invoke(pin);
            }
            else if (type == "TIMER" && commandId == "TIMER")
            {
                int timerNum = int.Parse(parts[2]);
                OnTimer?.Invoke(timerNum);
            }
            else if (_commandResponses.TryGetValue(commandId, out var tcs))
            {
                tcs.SetResult(line);
            }
            else
            {
                Console.WriteLine($"Unknown response ID: {commandId}");
            }
        }

        // Invokes the registered callback for an interrupt on a specific pin.
        private void HandleInterrupt(int pin)
        {
            if (_interruptCallbacks.TryGetValue(pin, out var callback))
            {
                callback();
            }
        }

        // Parses the response from a digital pin read command.
        private bool ParseDigitalReadResponse(string response, int pin)
        {
            var parts = response.Split(',');
            if (parts.Length == 5 && parts[1] == "D" && int.Parse(parts[2]) == pin)
            {
                return int.Parse(parts[3]) == 1;
            }
            throw new PCGPIOCommandException("Invalid digital read response.");
        }

        // Parses the response from an analog pin read command.
        private int ParseAnalogReadResponse(string response, int analogPin)
        {
            var parts = response.Split(',');
            if (parts.Length == 5 && parts[1] == "A" && int.Parse(parts[2]) == analogPin)
            {
                return int.Parse(parts[3]);
            }
            throw new PCGPIOCommandException("Invalid analog read response.");
        }

        // Parses the response from a bulk digital read command.
        private Dictionary<int, bool> ParseBulkReadResponse(string response, int[] pins)
        {
            var parts = response.Split(',');
            int pinCount = (parts.Length - 3) / 2; // Adjust for timestamp, type, and ID
            if (pinCount != pins.Length || parts[1] != "B")
                throw new PCGPIOCommandException("Invalid bulk read response.");
            var result = new Dictionary<int, bool>();
            for (int i = 0; i < pinCount; i++)
            {
                int pin = int.Parse(parts[i * 2 + 2]);
                int value = int.Parse(parts[i * 2 + 3]);
                result[pin] = value == 1;
            }
            return result;
        }

        // Ensures the digital pin number is valid based on controller configuration.
        private void ValidatePin(int pin)
        {
            if (_totalDigitalPins == -1)
            {
                throw new InvalidOperationException("Pin info not yet retrieved. Call ConnectAsync first.");
            }
            if (pin < 0 || pin >= _totalDigitalPins)
            {
                throw new ArgumentException($"Digital pin number must be 0 to {_totalDigitalPins - 1}.");
            }
        }

        // Ensures the analog pin number is valid based on controller configuration.
        private void ValidateAnalogPin(int pin)
        {
            if (_totalAnalogPins == -1)
            {
                throw new InvalidOperationException("Pin info not yet retrieved. Call ConnectAsync first.");
            }
            if (pin < 0 || pin >= _totalAnalogPins)
            {
                throw new ArgumentException($"Analog pin number must be 0 to {_totalAnalogPins - 1}.");
            }
        }

        // Ensures the timer number is valid based on controller configuration.
        private void ValidateTimer(int timerNum)
        {
            if (_timerCount == 0)
            {
                throw new InvalidOperationException("Timer info not yet retrieved. Call ConnectAsync first.");
            }
            if (timerNum < 1 || timerNum > _timerCount)
            {
                throw new ArgumentException($"Timer number must be 1 to {_timerCount}.");
            }
        }

        // Attempts to automatically detect the serial port connected to the controller.
        private string AutoDetectPort()
        {
            string[] ports = SerialPort.GetPortNames();
            foreach (var port in ports)
            {
                try
                {
                    using (var tempPort = new SerialPort(port, 1000000))
                    {
                        tempPort.Open();
                        tempPort.Write("PING\n");
                        string response = tempPort.ReadLine();
                        if (response.Contains("PONG"))
                        {
                            tempPort.Close();
                            return port;
                        }
                        tempPort.Close();
                    }
                }
                catch
                {
                    // Ignore errors and try the next port
                }
            }
            throw new PCGPIOConnectionException("Could not auto-detect controller.");
        }

        // Negotiates the optimal baud rate with the controller.
        private async Task<int> NegotiateBaudRateAsync()
        {
            int[] baudRates = { 1000000, 921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600 };
            foreach (var baud in baudRates)
            {
                try
                {
                    _serialPort.BaudRate = baud;
                    _serialPort.Open();
                    string commandId = Guid.NewGuid().ToString();
                    string command = $"T,{commandId}\n";
                    string response = await SendCommandAsync(command, commandId);
                    if (response.Contains(",T"))
                    {
                        string setBaudCommand = $"C,{baud},{commandId}\n";
                        await SendCommandAsync(setBaudCommand, commandId);
                        _serialPort.Close();
                        return baud;
                    }
                    _serialPort.Close();
                }
                catch
                {
                    // Ignore errors and try the next baud rate
                }
            }
            throw new Exception("Could not negotiate baud rate.");
        }

        // Pings the controller to verify it is responsive.
        private async Task PingAsync()
        {
            string commandId = Guid.NewGuid().ToString();
            string command = $"PING,{commandId}\n";
            string response = await SendCommandAsync(command, commandId);
            if (!response.Contains("PONG"))
            {
                throw new PCGPIOConnectionException("Controller did not respond to ping.");
            }
        }

        /// <summary>
        /// Disconnects from the controller and releases serial port resources.
        /// </summary>
        public void Dispose()
        {
            Disconnect();
            _serialPort?.Dispose();
        }
    }
}