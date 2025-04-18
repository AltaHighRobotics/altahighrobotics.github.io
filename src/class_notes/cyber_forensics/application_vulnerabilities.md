---
title: Application Vulnerabilities
parent: Cyberforensics
grand_parent: Class Notes
---
# Application Vulnerabilities
Cybersecurity Forensics Lesson 2.3.1

___
### Memory Injection
- Technique used to insert or manipulate code within a running process.
- Can be used maliciously
	- Code execution in security attacks
- Can be used positively
	- Debugging, customization, or enhancing program functionality
### Buffer Overflow
- Technical attack that is caused by poor coding and software development.
- Buffer overflow attacks happen when data overflows a buffer capacity.
	- Like pouring water into a pitcher beyond its capacity, causing a spill.
	- In code it results from inserting more data into a variable than it can hold, potentially leading to overwriting adjacent memory addresses.
### Integer Overflow
- Involve overflowing buffers with excessive data, potentially overwriting adjacent memory.
- Integer overflow attacks occur when numeric variables exceed their maximum value. Leading to unintended results like looping back to zero or causing system failures.
### Race Conditions
- Occurs in code when the execution path depends on the timing of data writes to shared memory, registers, or variables rather than their values.
- Caused by poor coding practices where shared memory isn’t locked before multiple processes attempt to write to it.
### Malicious Update
- Appears as a legitimate software update but contains harmful code or functionalities.
- Can be distributed through compromised servers or fake notifications, posing risks like unauthorized access, data theft,or system compromise.
- Users should verify updates from official sources to mitigate these risks.
