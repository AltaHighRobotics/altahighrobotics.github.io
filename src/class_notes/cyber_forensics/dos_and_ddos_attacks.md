---
title: DoS and DDoS Attacks
parent: CyberForensics 
grand_parent: Class Notes
---
# DoS and DDoS Attacks
Cybersecurity Forensics Lesson 2.4.8

___
### Denial of Service (DoS)  
- Disrupt the availability of services or information by  
	- Rendering them inaccessible  
	- Preventing communication  
	- Creating distractions  
- Impact network availability by overwhelming servers with traffic causing them to become unresponsive or slow down significantly.

### Distributed Denial of Service (DDoS)  
- Attacks involve multiple sources attacking a target simultaneously.  
	- Actual computers  
	- Servers  
	- IoT devices  
- More challenging to mitigate due to the distributed nature of the attack, making them more effective in overwhelming network resources.

### Amplified DDoS Attacks  
- Leverage servers or systems that generate a significantly larger response to a small request.  
- Commonly exploited protocols for amplification include  
	- <span style="color:rgb(255, 0, 0)">DNS</span> (Domain System Name)  
	- <span style="color:rgb(255, 0, 0)">NTP</span> (Network Time Protocol)  
	- <span style="color:rgb(255, 0, 0)">SNMP</span> (Simple Network Management Protocol)

### Reflected DDoS Attacks  
- Involve exploiting servers or systems to reflect attack traffic towards the target.  
- Attackers send requests with spoofed source IP addresses to these servers, making it appear as if the target is the source of the requests.  
- The servers then respond to the target with amplified traffic, reflecting the attack back to the intended victim.

### DDoS Attacks and Targets  
- <span style="color:rgb(255, 0, 0)">Network DDoS</span> attacks  
	- Target entire networks  
- <span style="color:rgb(255, 0, 0)">Application DDoS</span> attacks  
	- Target specific applications 
- <span style="color:rgb(255, 0, 0)">Operational technology (OT) DDoS</span> attacks  
	- Target physical machines connected to the network disrupting their operations.

### Defense strategies against DDoS attacks  
- Properly configuring public-facing servers  
- Monitoring connections  
- Ensuring proper load balancing and adequate bandwidth  
- Implementing security monitoring using IDS/IPS  
- Restricting the use of insecure protocols like ICMP and UDP.
