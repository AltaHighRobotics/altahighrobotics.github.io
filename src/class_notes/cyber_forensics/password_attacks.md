---
title: Password Attacks
parent: CyberForensics 
grand_parent: Class Notes
---
# Password Attacks
Cybersecurity Forensics Lesson 2.4.14

___
## Looking at the kali password:
Here is a very basic example of a password which is stored in the `/etc/shadow` directory
`kali:\$y\$j9T$ufXTBpN1QpgwlgqRFmb/B0\$/.y0ybAF4iNQXniErsDWf9QSl2HZH7LnBeRHB4ZiQa9`

Breakdown of above text:
`username:\$hash number\$salt\$hashed password:`

| Kali has the following data: | |
| --- | --- |
| username | kali |
| Hash Algorithm | '\$y\$' (or yescrypt)   |
| Salt: | ‘j9T$ufXTBpN1QpgwlgqRFmb/B0’   |
| Hash: | ‘/.y0ybAF4iNQXniErsDWf9QSl2HZH7LnBeRHB4ZiQa9’   |
| Plaintext password: | ‘kali’ |

## How to Defend Against a [[Brute Force Attacks|Brute Force Attack]]
- Strong Passwords, longer is better than complex
- Lockout after __ failed attempts  
- Two-Factor Authentication  

## How to Defend against [[Rainbow Tables|Rainbow Table Attacks]] 
- Salt those passwords!  
	- A salt is string of characters added to a password before it is hashed  
	- Using a unique salt for each user makes using a rainbow table more  
	difficult  
		- The rainbow table has to be recomputed for each user.  
		- If a password is found, which part is the hash and which is the password?  
- Key Stretching  
	- “Hashing the hash”  
	- Hashed values are hashed multiple times to increase the computation time required to hash each password

## How to Defend Against a [[Dictionary Attacks|Dictionary Attack]]
- Do not use generic passwords or old passwords  
	- Dictionary attacks use commonly-used passwords  
	- Dictionary attacks often contain old passwords that make have been compromised in the past  
- Strong Passwords  
- Increasingly longer delay between failed attempts  
- Lockout after __ failed attempts  
- Two-Factor Authentication  
