---
title: "CyberSecurity - Networking ALL"
parent: Cybersecurity Networking
grand_parent: Class Notes
---
# Networking Notes

___
## Vocab:
###### Multicast:
IPv6 protocol, a group can receive the packets but, in the end, only one device receives.
###### Unicast:
IPv4 and IPv6 protocol, sends the packets to one specific device.
###### Anycast:
IPv4 protocol, all the packets go to every device on the network.
###### Broadcast:
IPv4 and IPv6 protocol, sends the packets to a group of specific devices.
#cybersecurity/cast

###### Loopback addresses
What is the loopback address for IPv4 and IPv6, what does it allow a machine to do?

127.0.0.1 and 0:0:0:0:0:0:0:1; it allows a machine to send packets to itself for diagnostic purposes
#cybersecurity/loopback


Tunneling: Sending messages to an IPv6 network to all the nodes on the network.

Dual Stack: Allows devices to process IPv4 and IPv6 traffic simultaneously.

Shorthand notation: Allows for two networks/devices that use IPv6 to communicate over a network using IPv4.

Router advertisement: Uses the MAC address to create unique IP addresses that should not be used by other devices.

Stateless Address Auto-configuration: Allows an IPv6 address to be shortened since they can be very long to write out.

What is the purpose of a Domain Name System (DNS)?

The DNS, or Domain Name System, is the service that translates IP addresses to domain names.

How does DNS benefit end-users accessing websites?

Instead of users having to remember and type in the exact IP address of a web server, DNS converts it to a name so that a user can simply type in the URL; www.google.comis a lot easier to remember than 142.251.32.164.



### Clocks:
###### What is NTP?
Network Time Protocol has the important job of making sure all the clocks on the same network are synchronized.
##### What is a stratum?
A stratum is the level of accuracy of the clocks between systems



What are OIDs?

Object Identifiers are an identifier mechanism standardized by the International Telecommunications Union and ISO/IED for consistent naming.

Traffic Logs: Standard for sending/receiving notification messages.

Audit Logs: A document that records an event in an IT system.

Syslog: Displays an entry for the start and end of every session.

Hot Site: An exact replica of the original building

Warm Site: Not an exact replica, but also not an empty building

Cold Site: An empty building

What’s the difference between an active-active and an active-passive configuration?

An active-active configuration has every server active so the load balancer can spread across all servers.

An active-passive configuration only has some servers active, and the load balancer spreads across those active servers.

What is the VRRP?

The Virtual Router Redundancy Protocol is a computer networking protocol that provides for automatic assignment of available Internet Protocol routers to participating hosts.

What is a FHRP?

A First-Hop Redundancy Protocol is a computer networking protocol designed to protect the default gateway used on a subnetwork by allowing two or more routers to provide backup for that address.

### Server Downtime Acronyms
###### RTO:
The amount of time to recover processes after a disruption
###### RPO:
A measurement of the maximum tolerable amount of data to lose
###### MTTR:
The average time taken to restore a system when it does fail
###### MTBF:
The average time until a system failure
###### MTTF:
The average time until a system has an irreparable failure

## Cables
#### Twisted Pair Cables:
These cables have multiple wires that wrap around each other,
- ###### STP:
	Shielded Twisted Pair; Has protection against EMI by having a foil wrap around the wires
- ###### UTP:
	Unshielded Twisted Pair: Does not have protection, but is usually cheaper
#### Fiber-optic Cables:
Use optical fibers that carry light over far distances at light-speed, used for long-distance telecommunications and high-speed data in buildings
- ###### Single-mode:
	Carries only one type of light-wave to transfer data (between 1310 nm - 1550 nm)
- ###### Multimode:
	Carries multiple different light waves (red, green, and blue) to send more compact data

**What is an advantage of using fiber-optic cables over twisted-pair Ethernet cables?**
Fiber-optic cables transmit data at a higher speed, over a longer distance.

**What is a drawback of using fiber-optic cables over twisted-pair Ethernet cables?**
Fiber-optic cables are more expensive than twisted-pair cables, especially if the core of the cable is made of glass. They are also more difficult to install and repair than twisted-pair cables, and the equipment to install/repair is also more expensive.

What is the purpose of a patch panel?

The purpose of a patch panel is to make it easy to organize cables and make transitions when a device moves location.

What is a patch panel for fiber-optics known as?

Fiber distribution panel (FDP)

  
  

What are two benefits of having a patch panel? Scalability, easier maintenance, and they are inexpensive.

How do punchdown blocks create a connection?

They create connections by punching/pushing down wires into metal slots that strip the wire and complete a connection.

Name two common punchdown blocks?

66, 110, Krone, & Bix

Where does the word “Krone” come from?

The German word for crown

Management Plane: Controls the configuration of the user to device traffic

Application Layer: Contains the applications/programs used to set the network behavior

Control Layer: Translates and transmits all the data between these layers/devices

Infrastructure Layer: Networking devices that process or forward data on a network

What is DOCSIS?

The Data Over Cable Service Interface Specifications (DOCSIS) standard enables high-bandwidth data

transfer via existing coaxial cable systems that were originally used in the transmission of cable television

program signals

SSID: The WLAN network name

BSS: The collection of stations that communicate together in an 802.11 network

ESS: Interconnected WLANs integrated into LANs

What command can be used to determine the status of a router or switch?

Show Run

How are duplex and speed typically set?

To Auto

What are CRCs?

Cyclic Redundancy Checks are an error-detecting code commonly used in digital networks and storage devices to detect accidental changes to raw data.

What does an encapsulation error message indicate?

The router has a layer 3 packet it is trying to forward but is missing some element from the layer 2 header that is needed for forwarding the packet to the next hop.

What is an AUP?

An Acceptable Use Policy is a document identifying exactly what is and what is not appropriate activity

on an organization’s network

What is DLP?

Data loss prevention is a strategy to keep private or proprietary data such as health data, personally

identifiable information, financial statements, proprietary diagrams, and other private information from

being shared through unauthorized access.

What’s the difference between the MDF and an IDF?

The main distribution frame connects equipment to cables and subscriber carrier equipment. An

intermediate distribution frame serves as a distribution point for cables from MDF to individual cables

connected to equipment in areas remote from these frames.

NDA: A legal contract that identifies what information is confidential and limits its use and dispersal of that information.

MOU: A document detailing something both sides can agree to but may not be signed.

SLA: An agreement between two parties that indicate the minimum level of services required.
#
What information should be included in a network and network device baseline?

A class A1 data center, Between 59°F to 89.6°F, Between 20% to 80% humidity

In a client-to-site VPN connection, what two connection types are there?

Full tunnel and split tunnel

What is the difference between the two connection types in a client-to-site VPN connection?

In a full tunnel, all communication coming from the client would have to first go to the company’s VPN

concentrator to be routed accordingly, even traffic not destined for the company network. In a split

tunnel, only the traffic destined for the company’s internal network is sent to the VPN concentrator, all

other traffic is sent in a seperate unencrypted tunnel directly to its destination

What is the name of the microsoft created protocol used to perform remote functions on a Windows

system?

Remote Desktop Protocol (RDP)

What protocol does VNC use for remote connections?

Remote Frame Buffer (RFB) protocol

What is the purpose of MAC filtering?

MAC filtering uses that unique MAC address to identify devices connecting to the network and will either

allow or deny access to the wireless AP

 What is a wireless sniffer?

It is a device that allows a malicious actor to read packets sent to the wireless AP

Whar are four examples of IoT devices?

Smart refrigerators, baby cameras, video doorbells, smartwatches, wireless thermostats, etc.

How can network administrators best protect the network when IoT devices are introduced?

Keeping a detailed log of each device and the location of these devices, updating the default password,

and updating the firmware/patches to best protect them.

  
  
# 

A. Adversary-in-the-middle attack: Attacker positions themselves in between a user and their intended destination

B. DNS poisoning: Server has been altered and does not translate the proper IP address to a domain name

C. VLAN hopping: Malicious actor accessing ports that a typical system does not have access to

D. ARP spoofing: Device on a network lies about their MAC address

E. Rogue DHCP: Server impersonates a legitimate server, offering IP addresses and other network information

F. Rogue AP: Network device has been installed on a secure network without proper authorization

What is port tagging and how does it work? Port tagging is a standard method of frame tagging. This works by inserting a field into the frame to identify the VLAN. 802.1Q is the only option if we want to trunk between a Cisco switched link and another brand of switch.

How do PAgP and LACP differ? PAgP is proprietary, LACP is not so it can be used across multiple vendors.
#
What does it mean for something to be half-duplex? Full-duplex? Half-duplex means data can only be transmitted in one direction. Full-duplex can simultaneously transmit and receive information.

How does a system handle too much Ethernet traffic? Flow control via the pause frame, IEEE 802.3x, literally pauses network traffic to allow the system to “catch up”.

How can packets be captured? A physical tap (hardware) or a port mirror, a port redirection, or a switched port analyzer (SPAN) (software) can be used.

How can a user prevent packet capture and what are the two filtering methods? Port security can prevent unknown devices from forwarding packets and implement dynamic locking and static locking.

How does an Auto MDI-X work? An MDI-X works on newer network interfaces by detecting if the connection requires a crossover and automatically chooses the MDI or MDI-X configuration that appropriately matches the other end of the link.

Why do organizations use VLANs? To limit collisions on the network, to increase security, and to increase performance by limiting the number of network resources the packets travel across.

How do data VLANs and voice VLANs differ? A data VLAN is a VLAN that is configured to only carry user-generated traffic. A voice VLAN is configured to carry voice traffic from an IP phone.

What are the purposes of trunk ports and trunk links? Trunk ports can carry multiple VLANs at the same time. A trunk link is a 100 Mbps or 1 Gbps point-to-point link between two switches, a switch and a router, or a switch and a server.

What is the difference between an ARP table and a MAC address table? Trick question! They’re the same.

  

What are IEEE standards 802.3af and 802.3at? They are standards that describe how a powered device is detected and how power is given to a device from PoE and PoE+ technologies, respectively.

What is Spanning Tree Protocol and how does it work? STP is a link management protocol that provides path redundancy while avoiding looping in a network. STP uses the Spanning Tree Algorithm (STA) to create a topology database and search out and destroy redundant links.

What is NDP? Neighbor Discovery Protocol (NDP) operates at the link layer of the OSI model and is responsible for gathering necessary information for internet communication.

Routing Information Protocol (RIP) uses hop count as a routing metric to determine the most efficient route between the source and its destination using the Bellman-Ford algorithm.

Open Shortest Path First (OSPF) works by finding the best path from the source to the destination router according to its own shortest path first algorithm (Dijkstra).

Enhanced Interior Gateway Routing Protocol (EIGRP) uses the concept of an autonomous system to describe the set of contiguous routers running the same protocols and sharing the same information.

Border Gateway Protocol (BGP) is the core routing protocol of the internet and uses an algorithm to determine the best route.

What is the purpose of TTL? TTL limits the lifespan of data within a computer or network by limiting the amount of hops a packet can take before the data is discarded.

What is the purpose of SNMP? Simple Network Management Protocol (SNMP) is used quite extensively to monitor and control network devices.

What version(s) of SNMP encrypts, hashes and authenticates communications? SNMPv3

Match the following terms with their proper description.

Control Plane: Controls how data is sent from one place to another

Data Plane: Forwards the transmitted traffic

Management Plane: Configures, monitors, and provides management

What is the default VLAN for Cisco devices? VLAN1

What is the purpose of private VLANs? Private VLANs prevent users connected to the same network from communicating with one another.

What does port security protect the network against? Prevents unknown devices from connecting to a physical port, referred to as an interface, on a network switch.

  
# 
  

Why is it important to update patches and firmware as soon as reasonably possible? Updating patches and firmware is important to protect devices against potential malicious attackers because some patch and firmware updates are there to fix security vulnerabilities.

When updating device drivers and firmware, if the device is not operating as it previously was or as it should, as a network administrator what should you do? Rollback the installation, a rollback simply means removing the new driver and going back to the previous driver to get the device back working the way it previously was.

What does implicit deny mean? Implicit denial means that all traffic is denied unless it is specifically allowed by a rule.

What does explicit deny mean? Explicit denial allows all traffic unless it has been specially denied.

  
  
  

![](https://lh7-rt.googleusercontent.com/docsz/AD_4nXd4KcOcWk4C-cY0T5hvcbmDXR34sMD9A0u4RIvxEmsOkkKb9RD1VOK7jcCuW4TDYLKoE-hXAWfvBlML2_pm5M1RPr3wb6KaxXaWqn1_hQdG0N1LSPemRJWmX6rIFUdPEQ9ZY6af?key=hebBuxzRZRvvm15YV1G2QJZr)

#
What authentication protocol uses TCP and is considered more reliable than RADIUS? TACACS+

Which of the following is NOT a factor in multi-factor authentication? Something you eat

What protocol was developed by MIT in the 1980s and is integrated into Windows systems? Kerberos

Which authentication method requires certificates on both server and client sides? EAP-TLS

What type of authentication stores credentials directly on the device? Local authentication

Which protocol is considered a "lightweight" version of DAP? LDAP

What is the primary advantage of Single Sign-On (SSO)? One authentication for multiple resources

Which company created EAP-FAST? Cisco

What is the length of a Pre-Shared Key (PSK)? 256-bit

In 802.1X authentication, what is the device requesting access called? Supplicant

Which protocol was originally designed for dial-up users but is now used widely? RADIUS

What does PEAP require for authentication? Server-side PKI certificate only

Which protocol uses "tickets" for authentication? Kerberos

What is the main purpose of 802.1X? Port-based Network Access Control

Which companies collaborated to create PEAP? Cisco, Microsoft, and RSA

What type of transmission protocol does RADIUS use? UDP

Which authentication method is commonly used in home or small office settings? PSK

What protocol secures LDAP transmissions? SSL/TLS

How many factors are required for multi-factor authentication? More then 3 

Which protocol was specifically created for Cisco Systems devices? XTACACS

What distinguishes penetration testing from vulnerability scanning? It actively exploits vulnerabilities

In what type of penetration test do the pentesters know nothing about the system they are attacking AND the IT team of the system being attacked doesn't know the attack is coming? Double blind test

What was the root cause of Target's 2013 data breach? Compromised HVAC vendor credentials

What is the first crucial step in penetration testing? Defining scope

What type of test occurs when pentesters know nothing about the system but administrators expect the attack? Blind test

What is the purpose of a posture assessment? Check device status before network connection

How much did Target's 2013 data breach cost the company? $252 million

What is NOT typically included in a penetration test report? Employee personal information

  

What type of assessment analyzes current processes for future security planning? Process assessment

Why is a contract important for penetration testing? To protect both parties legally

What happens if a device fails a posture assessment? Quarantine or denied access

Which type of test provides pentesters with extensive system information? Target test

What assessment type evaluates third-party security risks? Vendor assessment

What is a primary purpose of the rules of engagement? Define test limitations

What can be an initial step in penetration testing? Vulnerability scan

What might cause a device to fail a posture assessment? Missing updates

What is the main goal of penetration testing? Simulated attack

Why are vendor assessments important? Security risk evaluation

What type of test provides the most realistic attack scenario? Double-blind test

When should penetration testing be performed? Regularly

What is the primary function of a SIEM system? To analyze raw network data in real-time

Which of the following is NOT typically logged by SIEM tools? Employee salaries

How do SIEMs assist online retail companies? By processing payments

What triggers automatic alerts in SIEM systems? Predefined metrics like incidents per hour

What role do security analysts play in SIEM operations? Investigating alerts

Which data source does a SIEM system monitor? Multiple sources including all mentioned

What benefit does real-time monitoring provide? Immediate threat detection

How do SIEMs help with compliance? By generating required reports

What type of website data can SIEM tools analyze? Both error codes and sales data

What view do SIEM tools provide to administrators? Real-time network state

What is the primary purpose of a change management process? To ensure changes are approved and implemented safely

Which of the following is NOT a category of disaster according to the text? Financial disasters

What is typically the maximum valid period for passwords in most organizations? 90 days

  

Which plan is more comprehensive than a disaster recovery plan? Business continuity plan

What is the first step in an incident response plan? Define the incident

What does DLP stand for? Data Loss Prevention

What type of policy defines appropriate activity on an organization's network? Acceptable Use Policy

During offboarding, what typically happens to user accounts? They are deactivated

What is one way to secure remote access connections? Network Access Control

What should be done with hardware during offboarding? Formally turned in and documented

Which of these is a form of system hardening? Removing unnecessary applications

What must all users who utilize technology in an organization sign? An AUP

Security policies are considered what type of documents? Living documents

What should be included in password requirements? Length and special characters

Which diagram shows how devices communicate with each other and includes elements like subnets and routing protocols? Logical network diagram

What is the primary purpose of a site survey? To examine a location and gather data for planning

Which distribution frame connects equipment to cables and subscriber carrier equipment? MDF

What type of agreement is considered the least formal and specific? MOU

What document is essential for troubleshooting electrical circuits and shows color coding of wires? Wiring diagram

Which agreement specifies the minimum level of services required between two parties? SLA

What should be included in all network device baselines? CPU, RAM, storage, and utilization

What type of diagram should include WiFi coverage information? Floor plan

Which legal contract specifically identifies and protects confidential information? NDA

What frame serves as a distribution point for cables from MDF to individual equipment? IDF

What is the primary purpose of NIC teaming? To provide backup if one NIC fails

In a switch cluster, what protocol is used for management? Cluster Management Protocol

Which of the following is NOT an approved fire suppression replacement for halon? Water

What is the recommended humidity range for a class A1 data center according to ASHRAE? 20% to 80%

Which disaster recovery site type requires the least initial investment? Cold site

What does FHRP stand for? First-Hop Redundancy Protocol

  

In an active-passive configuration, what happens when an active server fails? A passive server activates

What is the primary function of a UPS? Power backup

Which measurement indicates the average time until an irreparable system failure? MTTF

What is the temperature range recommended for a class A1 data center? 59°F to 89.6°F

What is the main advantage of a hot disaster recovery site? Minimal downtime

What type of redundancy helps maintain connectivity if an internet provider fails? ISP redundancy

Which metric measures the maximum tolerable amount of data loss? RPO

What is the primary purpose of switch stacking? Run as single switch

When was halon manufacturing banned by the EPA? 1994

What is the main benefit of power distribution units (PDU)? Remote power control

Which redundancy protocol provides automatic assignment of available IP routers? VRRP

What is the primary advantage of a cold disaster recovery site? Low maintenance cost

What type of configuration uses all servers actively for load balancing? Active-active

Which backup type only saves server configuration and not data? System state

What is the primary goal of a Denial of Service (DoS) attack? To disrupt service availability

A botnet is best described as: A network of remotely controlled computers

What is DNS poisoning? Modifying DNS translations

In an AiTM attack, the attacker: Positions themselves between user and destination

What is tailgating in the context of security? Following authorized personnel inside

A rogue access point is: A wireless AP installed without permission

What is phishing primarily used to obtain? Sensitive information

MAC spoofing involves: Cloning the router's MAC address

What type of attack is ransomware? Malware attack

An evil twin attack uses: Same SSID as legitimate network

What is shoulder surfing? Looking over someone's shoulder

A deauthentication attack results in: Network disconnection

Which is a characteristic of a DDoS attack? Multiple systems attacking

Piggybacking differs from tailgating because: The authorized person knowingly allows access

Which metric measures the maximum tolerable amount of data loss? RPO

What is the primary purpose of switch stacking? Run as single switch

When was halon manufacturing banned by the EPA? 1994

What is the main benefit of power distribution units (PDU)? Remote power control

  

Which redundancy protocol provides automatic assignment of available IP routers? VRRP

What is the primary advantage of a cold disaster recovery site? Low maintenance cost

What type of configuration uses all servers actively for load balancing? Active-active

Which backup type only saves server configuration and not data? System state

What is the primary goal of a Denial of Service (DoS) attack? To disrupt service availability

A botnet is best described as: A network of remotely controlled computers

What is DNS poisoning? Modifying DNS translations

In an AiTM attack, the attacker: Positions themselves between user and destination

What is tailgating in the context of security? Following authorized personnel inside

A rogue access point is: A wireless AP installed without permission

What is phishing primarily used to obtain? Sensitive information

MAC spoofing involves: Cloning the router's MAC address

What type of attack is ransomware? Malware attack

An evil twin attack uses: Same SSID as legitimate network

What is shoulder surfing? Looking over someone's shoulder

A deauthentication attack results in: Network disconnection

Which is a characteristic of a DDoS attack? Multiple systems attacking

Piggybacking differs from tailgating because: The authorized person knowingly allows access

What is a dictionary attack? Testing common passwords

IP spoofing involves: Modifying packet source addresses

VLAN hopping allows attackers to: Access unauthorized VLANs

A rogue DHCP server attack can cause: Service disruption

The best defense against phishing is: User education

An access control vestibule is designed to: Prevent tailgating

Which version of SNMP provides encryption, hashing, and authentication for secure communications? SNMPv3

What is the primary purpose of control plane policing? To protect against DoS attacks

What is the main function of Router Advertisement (RA) Guard? To block malicious RA messages

## Common protocols/ports:
###### SNMP:
Simple Network Management Protocol; it collects and manipulates valuable network information.
Ports:  **161** and **162**
###### SSH:
Secure Shell Protocol; Allows for a secure connection to network services over an unsecure network, mainly used with CLI functionality
Ports: **22**

### DNS commands:
###### A vs AAA:
Returns the domain name, reverse lookup.
###### CNAME
Alias for another domain name.
###### MX
IP addresses of the domain name.
###### SOA
Lists the server that contains the actual DNS information location.
###### PTR
How email addresses should be routed for the domain.
###### NS
Provides information about the administrator of the domain.
