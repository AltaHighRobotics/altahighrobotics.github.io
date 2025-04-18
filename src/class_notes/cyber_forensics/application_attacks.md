---
title: Application Attacks
parent: Cyberforensics
grand_parent: Class Notes
---
# Application Attacks
Cybersecurity Forensics Lesson 2.4.11 and 2.4.12

___
## Application Attacks Part 1
- Aimed at applications installed on systems with the intent to exploit vulnerabilities in the application or in the underlying system itself
- Common application attacks: - Replay
- Privilege Escalation - Directory Traversal

### Replay Attacks
- Uses a program to capture network packets when a victim is logging into an account
- These captured packets can be “replayed” or resent from the attacker to the site potentially giving them access to the account
- Requires many factors to work such as - Capturing the correct packets containing the info needed
- Packets that have been split up will need to be reassembled to achieve the same behavior as the original request

### Replay Attack Capturing Methods
- ARP Poisoning - Exploit the Address Resolution Protocol which provides mapping IP addresses to physical devices by using a compromised server
- Instead of being sent to the authentic site and being mapped properly, it will be sent to the attacker
- Tapping - Tapping or sniffing network traffic can pick up on the data being sent
- Malware - Some malicious programs have methods built in to collect packets

### Replay Attack Prep
- Before the actual replay attack occurs, the attacker would need to modify the data to ensure the packet contains the correct sending IP address and any other details that would make the attack appear legitimate.
- Once everything is in order they can carry out the attack

### Session Replay
- A session is the time that a user is authenticated and interacts with a web application
- A session replay attack occurs when an attacker has captured the session ID and any associated information and data allowing them to act as the user who has already logged in, essentially bypassing the login portion all together

### Pass the Hash
- Normally when a user logs into a system, the password is hashed, and the hash is then compared to the stored hash instead of the password itself being stored in plaintext
- In a pass the hash attack, the attacker has captured or used some method to gain that hash and is able to use it to gain access without using the actual password

### Defending Against Replay Attacks
- Using best practices and being safe while on the internet can assist in defending against these attacks.
- A nonce, a numerical value associated with the packet and validated when sent, can also aid in mitigation
- A common nonce used is a combination of the date and time - Since this value would constantly change, if a replay of the same packet occurs, then it would be discarded once the nonce was checked

### Privilege Escalation
- An exploit that allows a malicious actor to gain access to rights, information, or data they would normally not be able to have
- HTTP is stateless, meaning that when a user is logged in their device continuously communicates with the site or web application in a request-response cycle so the user can remain logged in while interacting with it
- By manipulating retransmitted data, an attacker could gain privileges

### Privilege Escalation Types
- Vertical Escalation - What is typically thought of in privilege escalation, where the attacker gains access to something higher, such as administrative rights
- Horizontal Escalation - Gaining access to information or data at the same level but perhaps with a different account
- In the case of an application, the web server account could be manipulated allowing an attacker to switch over to a normal user account on the system
- A web server account would normally be very limited, but a regular user account could have much more access

### Privilege Escalation Defense
- Install security updates, patches, and anti-virus software - Use address space layout randomization (ASLR), which is a memory protection process used to prevent exploits of memory corruption
- To prevent HTTP exploits - Only send session IDs to the client and keep critical info on the server
- Use digital signatures to secure data to the client - Encrypt the data sent to the client

### Directory Traversal
- Attempting to move from one folder to others within a system to access secure files or edits files and folders on the system
- Example: - You access an image on a website which is called by “filename=images/panda.jpg” and the image may have an absolute path to the file on the server as “/var/www/html/images/panda.jpg.” By manipulating the path using basic navigational commands, an unsecure system could be compromised. If the system were Linux based, typing in “filename=../../../etc/shadow” may give an attacker access to the password hashes.

## Application Attacks Part 2
- Aimed at applications installed on systems with the intent to exploit vulnerabilities in the application or in the underlying system itself
- Common application attacks: - Injection
- Buffer Overflow - Forgery

### Application Injection
- Take advantage of improper handling of user input by the application, allowing attackers to insert and execute unauthorized commands or code
- SQL Injection (SQLi): - Injection of malicious SQL (Structured Query Language) code into input fields or parameters of an application to manipulate the application's database queries and potentially gain unauthorized access to, modify, or delete data
- Cross-Site Scripting (XSS): - Malicious scripts are injected into web applications, and these scripts are then executed by the victim's browser, allowing attackers to steal user data, hijack sessions, or perform other malicious actions

### Application Injection cont’d
- Cross-Site Request Forgery (CSRF): - Fooling a user's browser into making an unwanted request to a web application on which the user is authenticated, allowing unauthorized actions such as changing account settings or making transactions.
- Command Injection: - Malicious commands are injected into input fields that are processed by the application as system commands, potentially leading to unauthorized access or data manipulation.
- LDAP Injection: - Targets Lightweight Directory Access Protocol (LDAP) queries, manipulating LDAP queries to gain unauthorized access to directory services or retrieve sensitive information.
- XPath Injection: - Malicious XPath expressions are injected into input fields that use XPath queries for XML data processing, enabling modification to XPath queries to extract sensitive information or manipulate XML data.

### Application Injection Mitigation
- Implement secure coding practices - Have input checks or validation
- Set parameters for queries, also called sanitizing inputs - Firewalls and additional tools can help detect and prevent intrusions
- Regular security and coding checks can address vulnerabilities

### Buffer Overflow
- Buffer overflow attacks happen when data overflows a buffer capacity.
- Like pouring water into a pitcher beyond its capacity, causing a spill.
- In code, it results from inserting more data into a variable than it can hold, potentially leading to overwriting adjacent memory addresses.
- Attackers can inject malicious code into the overflowed buffer, which has the potential of being executed in the adjacent memory The original data has been overwritten due to a buffer overflow in the name

### Request Forgery
- Cross-site request forgery (XSRF or CSRF) is an impersonation of a user performing an action on the website
- First thing, the user must be logged into the secure site they are wanting to access
- The attacker sets up a malicious link or site that contains a false request for the website the user is logged into
- The user accesses the malicious link or site, triggering a request to the secure site
- Since the user is already logged in, the request does not need validation, and the exchange occurs

### Request Forgery Types
- CSRF is a client-side request forgery leading to a compromise in the client’s data such as
- Unwanted bank transactions - Changes in credentials
- Other account manipulations - Server-side request forgeries occur when the request is meant to extract information or data from the server such as
- Accessing files - Altering the server’s behavior

### Request Forgery In Real Life
TikTok (2020): Attackers exploited a vulnerability that allowed them to send messages containing malware to TikTok users. This malware enabled CSRF attacks, causing other user accounts to submit requests on behalf of the attackers. TikTok patched the vulnerability within three weeks. 
### How did they do it?
Attackers exploited a combination of vulnerabilities, including a reflected XSS (Cross-Site Scripting) flaw and a CSRF vulnerability. The XSS flaw allowed attackers to inject malicious code into a URL parameter that wasn't properly sanitized. By combining this with the CSRF vulnerability, attackers created a JavaScript payload that triggered the CSRF issue and led to a one-click account takeover

### Request Forgery In Real Life
YouTube (2008): Princeton researchers discovered a CSRF vulnerability on YouTube that allowed attackers to perform nearly all actions on behalf of any user. This included adding videos to favorites, modifying friend/family lists, sending messages to a user’s contacts, and flagging inappropriate content. The vulnerability was quickly fixed.
### ING Direct (2008):
ING Direct, a banking website, had a CSRF vulnerability that allowed attackers to transfer money from users’ accounts, even though users were authenticated with SSL.


### Directory Traversal
- Attempting to move from one folder to others within a system to access secure files or edits files and folders on the system
- Example: - You access an image on a website which is called by “filename=images/panda.jpg” and the image may have an absolute path to the file on the server as “/var/www/html/images/panda.jpg.” By manipulating the path using basic navigational commands, an unsecure system could be compromised. If the system were Linux based, typing in “filename=../../../etc/shadow” may give an attacker access to the password hashes.
