---
title: Rainbow Tables
parent: CyberForensics 
grand_parent: Class Notes
---
# Rainbow Tables
Cybersecurity Forensics Lesson 2.4.14

___
## What is a Hash?  
- A hashing algorithm is an algorithm that converts input data (or a  
message) of varying size to a hash output of a fixed size  
- A hash is a one-way function, impossible to revert.  
- Generally, the longer the fixed output the less possibility of collisions  
(two inputs producing the same output), thus the more secure the  
hashing algorithm  

SomeBigLongValue      --->      33ce0634  
tinyval                            --->      0c430308
NggYuNgLYDngraaDY  --->      3803062f

## What is a Rainbow Table?  
- Pre-calculated series of hashes using known hashing algorithms  
- Commonly used for cracking passwords  
	- Find the matching hash string of text  
	- Look up the input text that gave the result  
	- Voila! There’s the password/input string  
- Rainbow tables are application-specific  
	- Built for each different application or OS  
	- No one table for all uses  

## How does a Rainbow Table work?  
- Get the first X characters of a hash  
	- Hash these characters  
- Get the first X characters of that hash  
	- Hash these characters  
- Do this repeatedly...  
	- This creates a "chain"  
	- Each chain can be referred to as a color "<span style="color:rgb(255, 0, 0)">red</span>" (first hash), "<span style="color:rgb(255, 152, 0)">orange</span>" (second hash), "<span style="color:rgb(255, 255, 0)">yellow</span>" (third hash), etc.  
- After obtaining enough chains, they create a table  
	- A table of all the colors... like a rainbow. Hence a "rainbow table".  
-  Only store the plaintext and final hash value for each chain  
	- All values in between plaintext and final hash can be re-computed as needed
  
- To use the table, take the first X characters of the target hashed password and look for a match in the table.
	- If a match is not found, take the first X characters, hash, and search again  
	- If a match is found, you know the plaintext at the front of that chain is part of the target password – this narrows the search by X characters.  
		- Take the next X characters and start the process again  
- It is a narrowing down of the thousand and millions of possibilities

## Rainbow Tables vs. Brute Force
  
- ###### Advantages of a Rainbow Table  
	- No need to match the whole string, looking for parts  
	- Not trying all values, only searching a table (fast)  
	- Can be done offline  
		- System does not know attempts are being made to crack the password of its users!  
- ###### Advantages of a Brute Force  
	- Does not need to store the large Rainbow Table dataset  
		- Which can be large! Can be gigs of text or even terabytes  
	- Works for all passwords, just takes time (lots and lots and lots of time)


## How to Defend against Rainbow Table Attacks  
• Salt those passwords!  
	• A salt is string of characters added to a password before it is hashed  
	• Using a unique salt for each user makes using a rainbow table more  
	difficult  
		• The rainbow table has to be recomputed for each user.  
		• If a password is found, which part is the hash and which is the password?  
• Key Stretching  
	• “Hashing the hash”  
	• Hashed values are hashed multiple times to increase the computation time required to hash each password
