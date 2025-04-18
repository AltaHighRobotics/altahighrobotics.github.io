---
title: Common Attack Surfaces
parent: CyberForensics 
grand_parent: Class Notes
---
# Command Attack Surfaces
Cybersecurity Forensics Lesson 2.2.1

___
### Threat Vectors  
- The path a malicious user takes to gain unauthorized access to a network, system, or data  
- Includes:  
	- Messages, files, removable devices, software, and much more Fake Google sign-in page

### Message-based  
- Exploits vulnerabilities in communication methods such as email, text or SMS, social media, and instant messaging  
- Usually contain a malicious link or attachment

### Images and Files  
- Image-based: Exploits vulnerabilities in image processing, display, or how they are shared  
	- Takes advantage of the inherent trust people have with visual content  
- File-based: Uses malicious code or scripting embedded within files that seem harmless  
	- Awareness of file sources, scanning files using updated anti-malware applications, and disabling macros can assist with this vector

### Voice Call-based  
- Uses phone systems and general human behavior to gain personal information about a target  
- Often involves the malicious actor pretending to be someone in need of information
- Verifying the caller’s identity, looking up unknown numbers, and being aware of pressure tactics should be used.

### Removable Device-based  
- Devices such as flash-drives, external drives, discs, memory cards, etc. can all serve as a pathway for a malicious attack to infiltrate a system.

### Vulnerable Software  
- Software itself can have unknown weaknesses or have pathways that allow an attacker to exploit  
- Two types  
	- Client-based – The user must install the software, usually unintentional or deceived  
	- Agentless – Do not require a user to install and can be zero-day attacks or exploit in popular or outdated software  
- Heartbleed is an example of a Zero- day software vulnerability

### Unsupported Systems and Applications  
- Unsupported software means it is no longer receiving patches and updates from the original vender.
- This can cause:  
	- Vulnerabilities remaining unchecked  
	- Easy and repeated exploits due to no changes being made  
- Upgrade immediately if possible  
- Isolate from other systems and sensitive data if needed

### Network-based Vectors  
- Exploit weaknesses in network or network-connected devices allowing an unauthorized user to:  
	- Access and steal data  
	- Disrupt operations  
	- Perform malicious actions on any system within the network  
- Can be divided into  
	- Wired  
	- Wireless, including Bluetooth

### Wired Attacks  
- Though they would seem secure when compared to wireless networks, wired networks can also be victims to malicious users.  
- Devices can be connected and act as legitimate network devices  
	- ARP Spoofing has an attacker clone the MAC Address of a network device allowing them to capture and redirect traffic  
	- DNS poisoning use compromised DNS servers to send users to sites or devices that are often masquerading as real ones

### Wireless Attacks  
- Exploits in wireless networks offer unique vulnerabilities and, in many cases, require the malicious user to mimic the wireless network(s) present  
- Rogue access points involve creating a fake access point that copy a real one in hopes of intercepting data  
- Other ways involve interrupting services on networks in hopes of forcing users to a rogue access point

### Bluetooth Attacks  
- Unsecured or poorly protected Bluetooth devices can lead to several issues including:  
	- Unsolicited messages  
	- Malicious files  
	- Loss of personal or sensitive data  
- In some cases, a malicious user may be able to take full control of the device.  
- Using strong passwords, turning off Bluetooth when not needed, and updating software can help to mitigate these attacks

### Open Ports and Services  
- Ports provide an entryway for certain processes to function within a system and network.  
	- Think of them as lanes on a highway but for network and data traffic  
- While some ports and services are crucial to the functionality of a system, ports and services that  only used temporarily should be closed/ turned off to prevent entry from a malicious user.

### Weak and Default Credentials  
- Often seen as the bane of any cybersecurity professional, weak or default passwords being used can lead to a host of problems.  
- These are usually found as  
	- Default credentials for software, hardware, or devices  
	- Guest devices or services that are connected to internal networks  
	- Weak passwords being used for accounts with higher privileges  
- Protection against these usually involves checking and changing default settings and hardening of anything that is public facing
- “admin” is often used as the default username and password of many devices.

### Supply Chain Attacks  
- Aimed at the third-party companies needed for an organization to function day to day  
- Often have access to organizations through physical or virtual means  
- Often hit at three different levels or areas  
	- Managed Service Providers  
	- Vendors  
	- Suppliers  
- Can lead to a cascading effect that hits several businesses

### Managed Service Providers (MSPs)  
- Typically provide services such as tech support or monitoring, cloud backups, or subscription services  
- Can have several organizations as clients which is often kept on a shared infrastructure, meaning one breach can lead to several organizations being compromised

### Vendors and Suppliers  
- Vendors tend to provide a finished product to an organization which could lead to security threats due to:  
	- Errors or flaws in software or hardware  
	- The vendor not being trustworthy and it being malware  
- Suppliers usually provide raw materials, components, or finished products to larger manufacturers leading to potentially the same issues as above for vendors, but effecting a much larger group

### Supply Chain Attack Mitigation  
- Organizations should always perform intensive security reviews of companies before entering in a contract.  
- Define expectations and security responsibilities for the company being hired for the product or service  
- Be mindful of the access to internal information and systems given to outside companies
