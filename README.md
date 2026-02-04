# ESP32 RTOS Proximity Warning System

An embedded proximity warning system built on the ESP32 using FreeRTOS.  
The system reacts to ultrasonic distance measurements and provides real-time
feedback through a servo motor, RGB LED, active buzzer, 16×2 LCD, and serial output.

The servo moves proportionally to detected distance, while the RGB LED and buzzer
indicate system state. System status and distance readings are displayed on both
the LCD and the serial monitor.

---

## System Tasks

- **Sensor Task** — Periodic ultrasonic distance sampling  
- **Servo Task** — Filtered, proportional servo actuation  
- **State Indicator Task** — RGB LED and buzzer control  
- **LCD Task** — Rate-limited user interface display  
- **Serial Task** — Debug and system logging  

Inter-task communication is handled using FreeRTOS queues and task notifications
to provide responsive behavior while minimizing CPU usage.

---

## Design Decisions

- Queue used as a mailbox to pass atomic system state snapshots  
- Task notifications used instead of polling to reduce unnecessary CPU wakeups  
- LCD updates rate-limited to prevent blocking real-time control tasks  
- Exponential smoothing and deadband applied to reduce servo jitter  
- Linear mapping used for proportional servo positioning  

---

## Challenges & Debugging

- LCD backlight brightness issues traced to improper current limiting and resolved through hardware-level tuning  
- LCD task initially caused system jitter due to blocking behavior and update frequency  
- Improved system responsiveness by rate-limiting LCD updates and adjusting task priorities  
- Servo jitter caused by sensor noise and timing, mitigated using exponential smoothing and deadband logic  
- Task starvation issues resolved through careful priority assignment and event-driven task notifications  

---

## Demo

[(https://youtube.com/shorts/Y-KnhP8CnVU?feature=share) ](https://youtube.com/shorts/Y-KnhP8CnVU?feature=share)
