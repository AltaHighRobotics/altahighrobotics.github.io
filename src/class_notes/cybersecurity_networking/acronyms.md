---
title: "CyberSecurity Net - Acronyms"
parent: Cybersecurity Networking
grand_parent: Class Notes
---
# Networking Acronyms

___
What are **OIDs**?
Object Identifiers are an identifier mechanism standardized by the International Telecommunications Union and ISO/IED for consistent naming.

What is the **VRRP**?
The Virtual Router Redundancy Protocol is a computer networking protocol that provides for automatic assignment of available Internet Protocol routers to participating hosts.

What is a **FHRP**?
A First-Hop Redundancy Protocol is a computer networking protocol designed to protect the default gateway used on a subnetwork by allowing two or more routers to provide backup for that address.
#cybersecurity/oid #cybersecurity/vrrp #cybersecurity/fhrp
### Network Address Assignment:
###### IP address:
Internet Protocol; A method of sending data across network devices based on logical addresses that are assigned to them
###### MAC address:
Details physical address of a network device, first 3 octets are manufacturing ID and the last 3 octets is the devices vendor ID (serial number), cannot be dynamically changed (like IP) but can be spoofed
###### DHCP:
Dynamic Host Configuration Protocol; Will automatically assign new network devices (like phones connecting to the wifi) a IP address so that the router can communicate with it
###### DNS:
Domain Name System; Used so that locating IP network locations could be done in a human-readable way using words rather than decimal IP addresses
###### ARP:
Address Resolution Protocol; Translates **IP** addresses **TO** their respective **MAC** addresses
IP >>> MAC
###### VLAN:
Virtual Local Area Network; Is used to separate the main LAN into different sub-networks that can be configured for different uses. Allows for communication from LAN to LANs in the network, but NOT from user to user in the network, users are more isolated. To limit collisions on the network, to increase security, and to increase performance by limiting the number of network resources the packets travel across.
###### PoE:
Power over Ethernet; A way for network devices to be powered AND connected to the network with a single twisted-pair cable. Is used to power IP phones, security cameras, and wireless APs.
IEEE standards: 802.3af, 802.3at, and 802.3bt; They are standards that describe how a powered device is detected and how power is given to a device from PoE and PoE+ technologies (different wattage amounts)
#cybersecurity/ip #cybersecurity/mac #cybersecurity/dhcp #cybersecurity/dns #cybersecurity/arp #cybersecurity/vlan #cybersecurity/poe
### Server Downtime Terms
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
#cybersecurity/rto #cybersecurity/rpo #cybersecurity/mttr #cybersecurity/mtbf #cybersecurity/mttf

### Wifi Acronyms
###### SSID:
The WLAN (wifi) network name
###### IoT:
Internet of Things: refers to the idea that many different appliances use the power of the internet to be managed, for things such as:
Smart refrigerators, baby cameras, video doorbells, smartwatches, wireless thermostats, etc.
###### AP:
Access Point: Used to connect a wired network to a wireless network using radio waves
###### BSS:
The collection of stations that communicate together in an 802.11 network
###### ESS:
Interconnected WLANs integrated into LANs

#wifi #cybersecurity/ssid #cybersecurity/iot #cybersecurity/ap #cybersecurity/bss #cybersecurity/ess

### Company Agreements
###### CRC:
Cyclic Redundancy Checks are an error-detecting code commonly used in digital networks and storage devices to detect accidental changes to raw data.
###### AUP:
An Acceptable Use Policy is a document identifying exactly what is and what is not appropriate activity on an organization’s network
###### DLP:
Data loss prevention is a strategy to keep private or proprietary data such as health data, personally identifiable information, financial statements, proprietary diagrams, and other private information from being shared through unauthorized access.
###### DA:
A legal contract that identifies what information is confidential and limits its use and dispersal of that information.
###### MOU:
A document detailing something both sides can agree to but may not be signed.
###### SLA:
An agreement between two parties that indicate the minimum level of services required.
#cybersecurity/crc #cybersecurity/aup #cybersecurity/dlp #cybersecurity/da #cybersecurity/mou #cybersecurity/sla


###### What’s the difference between the MDF and an IDF?
The Main Distribution Frame connects equipment to cables and subscriber carrier equipment. An Intermediate Distribution Frame serves as a distribution point for cables from MDF to individual cables connected to equipment in areas remote from these frames.

### Over-the-Network Management
###### RDP:
Remote Desktop Protocol
###### VNC:
Virtual Network Computing, is used to control network devices using the RFB protocol
###### RFB:
Remote Frame Buffer protocol is used for graphical control of user interfaces
#cybersecurity/rdp #cybersecurity/vnc #cybersecurity/rfb

### Network Routing:
###### RIP:
Routing Information Protocol; Uses hop count as a routing metric to determine the most efficient route between the source and its destination using the Bellman-Ford algorithm.
###### OSPF:
Open Shortest Path First; Works by finding the best path from the source to the destination router according to its own shortest path first algorithm (Dijkstra).
###### EIGRP:
Enhanced Interior Gateway Routing Protocol; Uses the concept of an autonomous system to describe the set of contiguous routers running the same protocols and sharing the same information.
###### BGP:
Border Gateway Protocol; Is the core routing protocol of the internet and uses an algorithm to determine the best route.
#cybersecurity/rip #cybersecurity/ospf #cybersecurity/eigrp #cybersecurity/bgp

### The unknowns

###### PAgP:
Port Aggregation Protocol; ***CISCO PROPRIETARY***, allows for the connection of multiple physical links to combine into a single logical link, is used to prevent collisions between devices by setting up a system of data diodes, where one ethernet cable to only transfer data one way, and another will send it back
###### LACP:
Link Aggregation Control Protocol; Works the same way as PAgP except it is not cisco proprietary, and can be used across different vendors


###### STP:
Spanning Tree Protocol; Is a link management protocol that provides path redundancy while avoiding looping in a network. STP uses the Spanning Tree Algorithm (STA) to create a topology database and search out and destroy redundant links.
###### NDP:
Neighbor Discovery Protocol; Operates at the link layer of the OSI model and is responsible for gathering necessary information for internet communication.




###### TTL:
Time To Live; Limits the lifespan of data within a computer or network by limiting the amount of hops a packet can take before the data is discarded.
###### SNMP:
Simple Network Management Protocol; Is used quite extensively to monitor and control network devices.
###### PDU:
Power Distribution Unit; Used for controlling electrical power in a data center, cannot be used for monitoring or remote management

###### TCP:
Transmission Control Protocol; A communications standard that enables application programs and computing devices to exchange messages over a network. Provides **lossless** data transmission
###### UDP:
User Datagram Protocol; A communications standard that enables application programs and computing devices to exchange messages over a network. Is used for higher speed data streams that do not require absolutely correct data. Provides a **lossy** data transmission
