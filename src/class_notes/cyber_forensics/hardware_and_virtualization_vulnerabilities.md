---
title: Hardware and Virtualization
parent: CyberForensics 
grand_parent: Class Notes
---
# Hardware and Virtualization
Cybersecurity Forensics Lesson 2.3.3

___
### Hardware
- Hardware vulnerabilities encompass weaknesses or flaws in the physical components of computer systems that could be exploited by attackers.
- Firmware vulnerabilities refer to security flaws in embedded software within hardware components.
- End-of-life hardware lacks support, updates, and patches from manufacturers, leaving systems vulnerable to new security threats. Attackers target systems with end-of-life hardware knowing that discovered vulnerabilities won’t be patched.

### Legacy hardware
- Older hardware lacking modern security features. - Susceptible to known vulnerabilities and incompatible with newer security measures.
- Attackers target legacy hardware due to its vulnerabilities and limited security capabilities.

### Mitigating hardware vulnerabilities
- Keep firmware up-to-date - Replacing end-of-life hardware
- Securely disposing of decommissioned devices - Conducting regular security audits
- Implementing additional security measures for legacy hardware

### Virtualization Vulnerabilities
- Refer to security weaknesses and risks associated with the use of virtualization technologies.
- VM escape
	- A vulnerability where an attacker gains unauthorized access from within a virtual machine to the host system or other VMs. It poses a risk by allowing attackers to compromise the security of the entire physical host or other co-located VMs.
- Resource reuse
	- Involve insecure handling of virtualized resources. Mitigation implementing strong isolation mechanisms between VMs, robust access controls, and careful management of shared resources.

### Resource reuse vulnerabilities
- Involve insecure handling of virtualized resources, leading to unauthorized access or information leakage between VMs.
- Mitigation involves implementing strong isolation mechanisms between VMs, robust access controls, and careful management of shared resources.

### Cloud-specific vulnerabilities
- Refer to the security weaknesses or risks that are unique to cloud computing environments.
- Inadequate identity management 
- Data breaches
- Insecure APIs - Shared technology issues
- Insufficient network security - Compliance and legal risks
- Lack of visibility and control
