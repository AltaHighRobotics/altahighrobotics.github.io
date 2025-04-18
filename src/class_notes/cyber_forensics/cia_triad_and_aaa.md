---
title: CIA Triad and AAA
parent: CyberForensics 
grand_parent: Class Notes
---
# CIA Triad and AAA
Cybersecurity Forensics Lesson 1.2.1

___
### Define the CIA Triad and explain why it is important in information security.
- Confidentiality
- Integrity
- Availability  
Provides a template of what needs to be done to ensure proper cybersecurity, for things such as data protection and transmission protection
- Access and privacy controls are designed to ensure only authorized users can access confidential information  
- They include the principles of identification, authentication, and authorization 
- Examples of authentication controls to protect include passwords, biometrics, PINs
### Give two examples of confidentiality controls and two examples of integrity controls.
Confidentiality controls:
- Requiring username+password to access sensitive data
- Making sure that people who are not employees are unable to configure system settings
Integrity controls:
- Keeping back-ups of your data so that it can be checked in case it was modified in any way
- Use proper databases that won't accidentally wipe data, use trusted vendors
### Explain the importance of non-repudiation in digital forensics and provide one method to achieve it.
- Non-repudiation allows for there to be undeniable proof of somethings rather that following a he-said-she-said. Can be accomplished with things such as cameras or logging systems
### Describe the AAA framework and its components.
- **Authentication** - Making sure that the person has the right amount of access
- **Authorization** - Giving only the amount of power that is needed for their tasks
- **Accounting** - Keeping records of what happened to provide evidence
### List three methods of authenticating people and three methods of authenticating systems.
- **Credential-Based**; Username and password, API keys  
- **Token-Based**; Hardware or software tokens  
- **Biometric**; Fingerprinting or behavioral metrics  
- **Mutual**; Client and server authentication  
- **Zero-Trust Architecture**; Assumes no system is trustworthy
### Briefly explain the five authorization models mentioned in the lesson.
###### The set of rules and policies that govern who can access and what actions can they perform within a system  
Determined by several factors:
	• System complexity  
	• Data sensitivity  
	• Compliance requirements  
	• Management  
	• Flexibility

## Authorization Model Types  
### Access Control Lists (ACL)  
- Assigns permissions directly to people or groups  
- Simple with smaller numbers, but can become complex with larger systems  
### Role-Based Access Control (RBAC)  
- Assigns permissions based on roles and users are assigned these roles  
- Examples include administrators, guests, editors, etc.  
### Attribute-Based Access Control (ABAC)  
- Assigns permissions based on attributes such as department, location, device type, time, etc.  
- Well-suited for complex systems with dynamic needs
### Rule-Based Access Control (RuBAC)  
- Rules define access conditions, often expressed as if-then statements  
- Allows specific security requirements  
- Very similar to ACLs
### Mandatory Access Control (MAC)  
- Enforces restrictions based on security labels to users and resources  
- Centrally controlled and often used in high-security environments
- High protection but less flexible for user needs
