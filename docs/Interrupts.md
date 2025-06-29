# Interrupt controllers and routing

On the Raspberry Pi family (especially Pi 3 and Pi 4), there are three distinct interrupt routing mechanisms, each with its own role and hardware scope.
Here's a breakdown, plus a note on one more subtle component you might find interesting:

✅ 1. ARM Local Timer and Interrupt Routing (QA7 Local Controller)
Base address: 0x40000000 (Pi 2/3)
Purpose: Routes per-core private interrupts, like:
ARM generic timers (CNTP, CNTV)
Mailboxes
Local timers
Used for: Direct-to-core interrupts, especially in bare-metal or SMP setups.
Not connected to the Broadcom controller or GIC.


✅ 2. Broadcom Interrupt Controller
Base address: 0x3F00B200 (Pi 2/3), 0xFE00B200 (Pi 4)
Purpose: Routes shared peripheral interrupts (e.g., UART, SPI, system timer).
Used in: Bare-metal, early boot, and Linux (on Pi 3 and earlier).
Limitations:
No per-core targeting.
No prioritization or advanced features.


✅ 3. ARM GIC (Generic Interrupt Controller)
Present only on: Raspberry Pi 4 and newer.
Purpose: Full-featured interrupt controller with:
Per-core routing
Prioritization
Software-generated interrupts (SGIs)
Used by: Linux and hypervisors on Pi 4+.


🧩 Summary Table
Controller	Base Address	Scope	Per-Core Routing	Present On
QA7 Local (ARM)	0x40000000	Private/core-local	✅ Yes	Pi 2/3
Broadcom	0x3F00B200	Shared peripherals	❌ No	All Pis
ARM GIC	0xFF840000 (typical)	Full system	✅ Yes	Pi 4+
