---
title: Account Configuration and Management
parent: Cybersecurity Linux
grand_parent: Class Notes
---
# Account Creation and Management
Cybersecurity Linux Lesson 2.2.2

___
### Account Creation and Configuration
Creation of a user leads to several actions and configurations to create a secure environment tailored to the user’s needs.  
• Typically, a Linux system will run a series of shell commands in Bash upon startup.  
• The system’s shell acts as an interpreter between the user and the operating system, reading startup configs and inputs.

### Account Authentication  
Prior to Bash running setup and configuration commands a user must be authenticated on the system  
• The `passwd` command is used when adding a user and setting up their password  
• The command can also be used to view password statues, set expirations, and more

### Password Directives  
• Along with the `passwd` command, the `chage` command can be used to change the expiration details of a user such as the age , expiration warning parameters, and the account expiration date  
• The `/etc/login.defs` file is associated with an account when created and sets password directives

### Failed Login Attempts  
• The `pam_tally2` and `faillock` utilities allow viewing and managing failed login attempts such as locking an account after so many attempts

### Data Associated with Authentication  
• System users and accounts created by applications are stored in the `/etc/passwd` file  
• Information includes:
	• Username  
	• User and group ID  
	• Home directories  
	• Default shell used  `
• The `/etc/group` file will store similar data, but with groups instead
• In the past, passwords were also stored in the `/etc/passwd` file; however, that has changed to now being stored in the `/etc/shadow` file, or the `/etc/gshadow` for group information  
• The password will not appear in plaintext, but is instead shown in a salted, hashed format

### Environment Configuration  
• The `/etc/profile` file is read when a user logs in and sets up the initial environment, such as system paths, shell prompts, aliases, and other functions
• `/etc/skel` is a directory that contains files and directories that are copied when an account is created  
• When the account is created, these provide the user with a pre-configured, or default, environment

### Bash Configuration  
• `.bash_profile` is initially executed upon logging in to setup the behavior and environment for the Bash shell  
• The `.bashrc` file has the same purpose but is applied immediately and only requires relaunching the terminal
