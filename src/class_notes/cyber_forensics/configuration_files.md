---
title: Configuration Files
parent: CyberForensics 
grand_parent: Class Notes
---
# Configuration Files
Cybersecurity Linux Lesson 1.7.1

___
## Updating Configuration Files
Updating configuration files involves making changes to the settings and parameters that define how a particular software or system operates  
- Configuration files are typically in plain text and contain instructions or configurations for different aspects of the software

## Procedures  
- Before changes are made, create a backup of the current configuration file  
- Use a text editor, like nano, to review the current settings  
- Make the requested modifications  
- The corresponding service may need to be restarted for changes to text effect  
- Test the new configurations, monitor for issues, and document for future reference

## RPM-based Systems  
- When updating packages in RPM-based systems, when a configuration file has been modified and includes an update, the new version may be saved with a `.rpmnew` extension to avoid overwriting user changes  
- The original file may be saved with a `.rpmsave` extension

## Repository Configuration Files  
- Files that contain settings and configurations specific to a version control system repository  
- Configuration files include:  
	- `/etc/apt.conf` in Debian-based systems  
	-  `/etc/yum.conf` in RPM-based systems  
	- `/etc/dnf/dnf.conf` in Fedora and CentOS systems  
- The `/etc/yum.repos.d` directory contains individual repository configuration files with each file corresponding to a specific repository and including settings for package sources  
- The `/etc/apt/sources.list.d` directory contains additional configuration files for software repositories
