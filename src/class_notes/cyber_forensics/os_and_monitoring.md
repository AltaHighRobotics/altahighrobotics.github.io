---
title: OS and Monitoring
parent: CyberForensics 
grand_parent: Class Notes
---
Cybersecurity Forensics Lesson 4.5.2

___
# OS and Monitoring
### Operating System Security
- Measures and practices implemented to protect the operating system and its components from unauthorized access, attacks, and potential security threats.
- Involves the implementation of security controls, policies, and mechanisms to ensure confidentiality, integrity, and availability of the OS.
- Two components commonly associated with OS security: 
	- Group Policy
	- SELinux

### Group Policy
- A feature in Microsoft Windows OS for defining and enforcing security settings and configurations.
- Enables centralized management of security policies, application settings, and other configurations in Active Directory.
- Group Policy settings control user access, password policies, software installation, and security-related configurations.
- Contributes to a more secure OS environment by enforcing consistent policies across the network.

### SELinux
- A security extension in the Linux kernel for mandatory access controls (MAC).
- Surpasses traditional discretionary access controls (DAC) by enforcing policies.
- SELinux policies define rules for interactions between users, processes, and files.
- These policies enhance the overall security of the Linux OS.

### Security Measures – File Integrity Monitoring (FIM)
- Monitors and detects changes to files and file systems. - Ensures integrity of critical systems and application files.
- Helps detect and respond to security incidents like unauthorized access or malware infections.

### Data Loss Prevention (DLP)
- Prevents unauthorized access, sharing, or leakage of sensitive data.
- Monitors and controls data transfers within the organization. - Includes content discovery, encryption, and polices to protect sensitive data.

### Network Access Control (NAC)
- Regulates and restricts network access based on predefined policies.
- Allows only authorized devices and users to connect to the network.
- Assesses device health and compliance before granting access.

### Endpoint Detection and Response (EDR)
Extended Detection and Response (XDR)
- Detects, investigates, and responds to security incidents at the endpoint level.
- EDR monitors and analyzes endpoint activities for signs of malicious behavior.
- XDR integrates data from multiple security layers for a comprehensive threat view.

### User Behavior Analytics (UBA)
- Analyzes patterns of user behavior to identify anomalies and security threats.
- Detects insider threats, compromised accounts, and other security incidents.
- Monitors user activities and provides alerts for further investigation

### Combining Measures for Cybersecurity
- FIM ensures file integrity. - DLP prevents data loss.
- NAC controls network access. - EDR/XDR detects and responds to endpoint threats.
- UBA analyzes behavior for security insights. - Combined, these measures contribute to a robust cybersecurity posture.
