# HID Interrupt IN Transfer Implementation

This implementation adds interrupt IN transfer functionality to the USB HID driver, allowing for more efficient keyboard and mouse input handling compared to traditional polling methods.

## Functions Added

- `HIDStartInterruptIN` - Performs a single interrupt IN transfer
- `HIDStopInterruptIN` - Stops interrupt IN transfers (placeholder)
- `HIDReadInterruptReport` - Convenient wrapper for interrupt IN transfers
- `HIDGetInterruptInterval` - Gets the device's preferred polling interval
- `HIDEnableInterruptIN` - Configures device for optimal interrupt IN operation
- `HIDEnableInterruptINSimple` - Easy setup with sensible defaults

## Benefits Over Polling

### Traditional Polling Approach:
- **High CPU Usage**: Constantly polls device even when no input is available
- **Fixed Timing**: Uses arbitrary delay intervals (typically 10ms)
- **Inefficient**: Wastes USB bus bandwidth with unnecessary transfers

### Interrupt IN Approach:
- **Event-Driven**: Only transfers data when device has new input
- **Device-Optimized**: Uses the device's preferred polling interval
- **Efficient**: Reduces CPU usage and USB bus traffic

## Example Usage

```c
// Get keyboard device number (from enumeration)
uint8_t keyboardDevice = 1; // Example device number

// Buffer for keyboard report (typically 8 bytes for boot protocol)
uint8_t keyboardData[8];
uint32_t bytesRead;

// Get device's preferred polling interval
uint8_t pollInterval;
if (HIDGetInterruptInterval(keyboardDevice, 0, &pollInterval) == OK) {
    LOG("Keyboard polling interval: %d ms\n", pollInterval);
}

// Main input loop
while (true) {
    // Read interrupt report - blocks until device has data
    if (HIDReadInterruptReport(keyboardDevice, 0, keyboardData, 8, &bytesRead) == OK) {
        // Process the keyboard data
        ProcessKeyboardInput(keyboardData, bytesRead);
    }
    
    // Optional: small delay based on device's preferred interval
    Cpu::DelayInMicroseconds(pollInterval * 1000); // Convert ms to microseconds
}
```

## Implementation Notes

- The implementation automatically finds the interrupt IN endpoint for HID devices
- Uses the existing `HCDChannelTransfer` function for the actual USB transfer
- Properly sets up the USB pipe with correct endpoint address and packet size
- Includes comprehensive error checking and logging
- Maintains compatibility with existing HID functions
- **Fixed transmission errors by implementing proper USB data toggle handling**

## Error Codes

The functions return standard `RESULT` error codes:
- `OK` - Success
- `RESULT::ErrorArgument` - Invalid parameters
- `RESULT::ErrorDeviceNumber` - Invalid device number
- `RESULT::ErrorNotHID` - Device is not a HID device
- `RESULT::ErrorIndex` - Invalid HID index
- `RESULT::ErrorDevice` - No interrupt IN endpoint found

## Troubleshooting

### "Transmission Error" Issues
**Problem**: Calls to `HIDStartInterruptIN` result in transmission errors.

**Root Cause**: USB interrupt endpoints require proper data toggle handling (alternating between DATA0 and DATA1 PIDs on successive transfers).

**Solution**: The implementation now includes automatic data toggle tracking:
- Each interrupt endpoint maintains its data toggle state
- PIDs alternate between `USB_PID_DATA0` and `USB_PID_DATA1` automatically
- Data toggle is updated after each successful transfer
- This resolves the `DataToggleError` that was causing transmission failures

**Technical Details**:
- The USB specification requires data toggle synchronization for reliable transfers
- The DesignWare USB controller detects data toggle mismatches and reports them as transmission errors
- Our implementation stores the data toggle state in the endpoint descriptor and updates it after each transfer

## Enabling Interrupt IN Communication

While most standard USB HID devices work immediately with interrupt IN transfers after enumeration and configuration, some devices benefit from explicit optimization settings:

### Do I Need to "Enable" Interrupt IN?

**For most standard HID devices: NO** - interrupt IN endpoints are automatically enabled when the USB device is configured during enumeration. The basic interrupt IN functions (`HIDStartInterruptIN`, `HIDReadInterruptReport`) will work immediately.

**However, some devices may benefit from explicit configuration:**

### When to Use Enable Functions

Use `HIDEnableInterruptINSimple()` or `HIDEnableInterruptIN()` if:

1. **Device-specific requirements**: Some devices require explicit protocol or idle settings
2. **Optimization**: You want to reduce unnecessary polling by setting idle rates
3. **Protocol selection**: You need boot protocol vs report protocol for compatibility
4. **Troubleshooting**: The device doesn't respond to interrupt IN transfers initially

### Enable Function Options

#### Simple Enable (Recommended)
```c
// Use sensible defaults: report protocol, idle on change only
RESULT result = HIDEnableInterruptINSimple(keyboardDevice, 0);
if (result != RESULT::Ok) {
    printf("Failed to enable interrupt IN: %d\n", result);
}
```

#### Advanced Enable (Full Control)
```c
// Full control over protocol and idle settings
RESULT result = HIDEnableInterruptIN(
    keyboardDevice,     // Device number
    0,                  // HID index
    true,               // Set protocol
    1,                  // Report protocol (0=boot, 1=report)
    true,               // Set idle rate
    0                   // Idle rate: 0=send only on change
);
```

### Protocol Types

- **Boot Protocol (0)**: Simple, standardized format (limited features)
- **Report Protocol (1)**: Full feature set, device-specific reports (recommended)

### Idle Rate Settings

- **0**: Only send reports when state changes (most efficient)
- **1-255**: Send reports at regular intervals (value × 4ms), even if no change

### Enable Function Details

```c
RESULT HIDEnableInterruptIN (
    uint8_t devNumber,      // Device address
    uint8_t hidIndex,       // HID configuration index
    bool setProtocol,       // Whether to configure protocol
    uint8_t protocolValue,  // 0=boot, 1=report protocol
    bool setIdleRate,       // Whether to configure idle rate
    uint8_t idleRate        // 0=change only, >0=interval in 4ms units
);

RESULT HIDEnableInterruptINSimple (
    uint8_t devNumber,      // Device address  
    uint8_t hidIndex        // HID configuration index
);
```

**Note**: These functions will log warnings for unsupported requests but continue operation, as many devices work fine without explicit enable commands.

This implementation provides a solid foundation for efficient HID input handling while maintaining compatibility with the existing codebase.

## Complete Example with Enable Function

Here's a complete example that shows proper interrupt IN setup:

```c
#include "HidUsbDevices.h"
#include "Timer.h"

void KeyboardInterruptExample() {
    uint8_t keyboardDevice = 2; // Assume keyboard is device 2
    uint8_t keyboardData[8];
    uint32_t bytesRead;
    
    // Step 1: Optional - Enable interrupt IN with optimal settings
    printf("Enabling interrupt IN for keyboard...\n");
    RESULT enableResult = HIDEnableInterruptINSimple(keyboardDevice, 0);
    if (enableResult != RESULT::Ok) {
        printf("Warning: Could not optimize interrupt IN settings: %d\n", enableResult);
        printf("Continuing anyway - device may still work...\n");
    }
    
    // Step 2: Get the device's preferred polling interval
    uint8_t interval;
    if (HIDGetInterruptInterval(keyboardDevice, 0, &interval) == RESULT::Ok) {
        printf("Device interrupt interval: %d ms\n", interval);
    }
    
    // Step 3: Use interrupt IN transfers for efficient input handling
    printf("Starting keyboard monitoring with interrupt IN...\n");
    
    while (true) {
        // Perform interrupt IN transfer - only completes when device has data
        RESULT result = HIDReadInterruptReport(keyboardDevice, 0, 
                                             keyboardData, sizeof(keyboardData), 
                                             &bytesRead);
        
        if (result == RESULT::Ok && bytesRead > 0) {
            printf("Key event: ");
            for (uint32_t i = 0; i < bytesRead; i++) {
                printf("%02x ", keyboardData[i]);
            }
            printf("(%d bytes)\n", bytesRead);
            
            // Process keyboard data here
            ProcessKeyboardData(keyboardData, bytesRead);
        } else if (result != RESULT::Ok) {
            printf("Interrupt IN transfer failed: %d\n", result);
            Cpu::DelayInMicroseconds(10000); // Brief delay before retry
        }
        
        // No delay needed here - interrupt IN is event-driven!
        // The transfer above blocks until the device has new data
    }
}

// Alternative: Advanced enable with custom settings
void KeyboardInterruptAdvancedExample() {
    uint8_t keyboardDevice = 2;
    
    // Enable with custom settings
    RESULT result = HIDEnableInterruptIN(
        keyboardDevice,     // Device number
        0,                  // HID index  
        true,               // Set protocol
        1,                  // Report protocol (full features)
        true,               // Set idle rate
        10                  // Idle rate: 10 * 4ms = 40ms max interval
    );
    
    if (result == RESULT::Ok) {
        printf("Interrupt IN enabled with custom settings\n");
    }
    
    // Continue with interrupt transfers...
}
```

### Troubleshooting Enable Issues

If the enable functions fail:

1. **Check device support**: Not all devices support SetProtocol or SetIdle
2. **Try without enable**: Many devices work fine without explicit enable calls
3. **Check logs**: The functions log detailed information about failures
4. **Use simple enable first**: Try `HIDEnableInterruptINSimple()` before custom settings

```c
// Robust approach with fallback
RESULT EnableInterruptINRobust(uint8_t device, uint8_t hidIndex) {
    // Try simple enable first
    RESULT result = HIDEnableInterruptINSimple(device, hidIndex);
    
    if (result != RESULT::Ok) {
        printf("Simple enable failed, trying manual protocol setup...\n");
        
        // Try just setting protocol
        result = HIDEnableInterruptIN(device, hidIndex, true, 1, false, 0);
        
        if (result != RESULT::Ok) {
            printf("Protocol setup failed, using device defaults\n");
            // Device will likely still work with defaults
        }
    }
    
    return RESULT::Ok; // Continue regardless - most devices work anyway
}
```
