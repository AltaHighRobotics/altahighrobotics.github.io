---
title: Supply Chain Vulnerabilities
parent: CyberForensics 
grand_parent: Class Notes
---
# Supply Chain Vulnerabilities
Cybersecurity Forensics Lesson 2.3.4

___
### Supply Chain Vulnerabilities
- Weaknesses or risks associated with products and services supplied by third- parties
- Can be exploited leading to a compromise to the entire organization
- Key vulnerability points: 
	- Service
	- Hardware 
	- Software

### Service Provider Vulnerabilities
- Many organizations contract out various IT services such as cloud computing, management services, and security.
- An exploited service may lead to unauthorized access to customer data, disruptions in service, or more
- Being meticulous in selecting a service provider, performing security checks, and having strong contractual agreements can assist in limiting these

### Hardware Provider Vulnerabilities
- Responsible for manufacturing and supplying the physical components of computer systems such as routers, servers, etc.
- Weaknesses can compromise an entire system - Attackers could install malware, access and manipulate data, or gain unauthorized access to other system components
- Mitigation includes validating the integrity of the components, working with trusted vendors, and conducting security assessments of the hardware.

### Software Provider Vulnerabilities
- Develop and distribute applications, operating systems, and other software
- Vulnerabilities in these systems can lead to unauthorized access, insertion of malicious code, or compromising data
- Mitigation includes regular security updates, installing patches, using reputable vendors, performing security checks, and monitoring for vulnerabilities

### Combating Supply Chain Vulnerabilities
- Because of how supply chains are woven together, creating a complex system, approaching risk management needs to have a comprehensive approach including:
- Vendor Risk Management:
	- Assessing the security of suppliers, service providers, and others in the supply chain
- Continuous Monitoring:
	- Ongoing monitoring for security issues, updates, alerts, and changes
- Incident Response Planning: 
	- Develop and regularly test response plans that address potential supply chain issues
- Security Standards and Certification: 
	- Encourage or require suppliers recognize security standards and obtain relevant certifications

### Cryptographic Vulnerabilities
- Refer to weaknesses in the design, implementation, or use of cryptographic systems that could be exploited by attackers
- Cryptography is highly utilized in securing information particularly against unauthorized access, data breaches, and any other compromise in integrity

### Key Management and Weak Algorithms
- Key Management
	- Weaknesses can arise during generation, distribution, or storage of keys
	- If cryptographic keys are compromised, data could be decrypted and accessed
- Weak Algorithms
	- Using outdated or insecure algorithms allow attackers to possibly break the encryption, decrypt information, or perform other attacks

### Randomness and Side-channel Attacks
- Random Number Generation Weakness
	- Insecure or predictable number generation can make the encryption process weaker and susceptible to brute force attacks
- Side-channel Attacks
	- Exploited information leaked from the execution of cryptographic algorithms such as timing, power consumption, or electromagnetic readings can give insight to cryptographic keys and compromise their integrity

### Padding Oracle and Key Length/ Entropy
- Padding Oracle Attacks - Padding schemes used in protocols like SSL and TLS can be exploited through the error handling process
- Key Length and Entropy Issues - Using shorter keys or inadequate entropy in generating keys makes them more susceptible to brute-force attacks

### Backdoors and Post-Quantum Concerns
- Cryptographic Backdoors
	- The deliberate or unintentional inclusion of vulnerabilities in cryptographic systems allowing access to systems and compromises the integrity of the encrypted data
- Post-quantum Cryptography Concerns
	- Future quantum computing could outclass current cryptographic algorithms, reemphasizing the need for post-quantum cryptographic standards

### Mitigating Cryptographic Vulnerabilities
- Implementing secure practices and staying informed of emerging threats
- Use well established cryptographic algorithms - Maintain strong key management practices
- Implement proper number generation techniques - Monitor for side-channel attacks and implement countermeasures
- Stayed informed of best practices and standards

### Misconfiguration Vulnerabilities
- Security weaknesses that arise from errors or oversights in software, systems, networks, and/or applications
- Can lead to a multitude of security risks including exploits to the system and data compromising
- Can result from:
	- Human error
	- Lack of awareness
	- Inadequate security practices

### Types of Misconfiguration Vulnerabilities
- Using default settings, such as factory credentials, can allow attackers to gain access to systems
- Weak credentials expose systems to brute force attacks
- Leaving ports or service open/ running can provide an attack surface for malicious users
- Inadequate access control can allow unauthorized access
- Failure to update software, configure devices, establish proper firewall rules, and more can all lead to vulnerabilities

### Handling Misconfiguration Vulnerabilities
- Adopt a robust security plan that includes: - Regular security audits
	- Ensure default settings have been changed and/ or are secure
	- Implement access control properly and have a password policy in place
	- Keep all software, systems, applications, etc. updated
	- Provide security training and awareness for employees
	- Address any potential issues from assessments and monitoring to enhance the security of an organization

### Mobile Vulnerabilities
- Security risks associated with smartphones, tablets, and other mobile devices and compromising the data stored or transmitted by the device
- Two common types of mobile vulnerabilities
	- Side-loading
	- Jailbreaking or Rooting
### Mobile: Side-loading
- Installing applications on a mobile device from a source other than the official app stores – e.g. Google Play Store, Apple App Store
- These unofficial apps may be malicious and contain code that compromises the security of the device
- The solution is only downloading apps from trusted sources

### Mobile: Jailbreaking and Rooting
- Removing software restrictions from the manufacturer or operating system to gain escalated privileges and access to system files
- Jailbreaking (Apple iOS) and Rooting (Android) tends to give users more customization and control over devices, it does bypass built-in security features and prevents it from receiving automatic security updates
- Keeping native settings and operating systems is key to keeping the integrity of the device

### Zero-day Vulnerabilities
- Security vulnerabilities in software, hardware, or firmware that are unknown to the vendor or developer and therefore have not been patched or addressed
- Designated as zero-day because once exploited or discovered, there are zero days for the developer to release a patch.

### Zero-day Nightmares
- Because the vulnerability is unknown by vendors, there several issues to contend with
	- No patches or fixes available to address it
	- Being unknown, there is a high chance there is no way to detect it or block attacks using the exploit
	- Often heavily planned by larger hacking groups, state actors, or other malicious entities to launch coordinated attacks to steal information and compromise systems
	- The time between discovery and patching is crucial as well as how fast applying the update can be performed as systems are vulnerable during this process

### Alleviating Zero-day Attacks
- Since zero-day attacks take advantage of unknown weaknesses, it is important to be proactive in defending against them such as:
	- Employ strong security practices across the board
	- Segmenting networks to prevent entire systems from being exposed
	- Implementing the principle of least privilege
	- Use of intrusion detection systems
	- Stay informed of emerging vulnerabilities and threats
	- Regularly update, and in the case of zero-day attacks stay informed of release times for patches addressing such vulnerabilities
