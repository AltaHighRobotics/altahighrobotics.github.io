---
title: File and Directory Management
parent: Cybersecurity Linux
grand_parent: Class Notes
---
# File Directory Management
Cybersecurity Linux Lesson 1.2.5

___
Users will learn to navigate and manage the files  
and directories contained within the Linux  
system  
- Linux systems have a resourceful CLI that contains multiple file and directory operations
	- These allow  
		• Creation  
		• Deletion  
		• Moving  
		• Copying  
		• And other manipulations of files and directories

### Navigating a Linux System
Several commands assist users in knowing where they are and what files are located there while traversing the file directory:

#### **<span style="color:rgb(255, 0, 0)">pwd</span>** (print working directory):
Shows the users current location while  
working in the directory  
#### **<span style="color:rgb(255, 0, 0)">ls</span>** (list):
Shows the files and directories located in the current  
working directory.
Like many command in Linux systems, list has additional options that can be added to include more information  
• `ls –a` shows hidden files/directories  
• `ls -l` shows the long format  
• `ls –R` shows recursive files/directories  
The list command has many additional options that can be explored
#### **<span style="color:rgb(255, 0, 0)">tree</span>** 
The tree command will print a visual map of the current working directory as well as any subdirectories  
• `tree -d` will only list directories  
• As with the list command, `tree` offers other options that can be explored as well using the command `tree -- help`
#### **<span style="color:rgb(255, 0, 0)">cd</span>**
The change directory command allows users to navigate around  
• `cd <Directory Name>` moves the user to the listed directory  
• Using the absolute or relative path moves you to that directory without going step by step as seen in the command `cd Documents/Sub_Directory`
• `cd ..` moves the user up one directory  
• `cd` by itself will return the user to the home directory (~/)
#### **<span style="color:rgb(255, 0, 0)">mkdir</span>**
The make directory command allows users to create new directories within the terminal
• `mkdir <Directory_Name>` will create a directory within the current working directory  
• `mkdir /Documents/Test` will create a directory within the Documents folder  
• The command can be used with relative or absolute paths
#### **<span style="color:rgb(255, 0, 0)">touch</span>**
The touch command allows users to create new files within the terminal  
• `touch <File1>` will create a file within the current working directory  
• `touch <File2> <File3>` will create two files within the current working directory and can be extended as needed to include more files
• `touch /Documents/File4` will create a file within the Documents folder
#### <span style="color:rgb(255, 0, 0)">**cat**</span>
The cat, or concatenate command allows users to view files within the terminal  
• `cat <File>` will print the contents of the file into the terminal
#### **<span style="color:rgb(255, 0, 0)">mv</span>**
The move command can move files or directories wherever specified  
• `mv <file/directory_name> <new_location>` will move the file to the designated location  
• When used without a designated new location the move command can be used to rename a file or directory in the current working directory if a new name is added instead such as seen with `mv <file> <new_file_name>`
#### **<span style="color:rgb(255, 0, 0)">cp</span>**
The copy command can copy files or directories wherever specified  
• `cp <file/directory_name> <new_location>` will copy the file/directory to the designated location  
• When copying a directory, the `–r` option must be included to include the files within the directory unless it is empty
#### **<span style="color:rgb(255, 0, 0)">rm</span>**
The remove command is used to delete files or directories  
• `rm <file_name>` deletes a file or directory  
• The remove command will only remove a directory if the `–r` option is added
• Adding `–i` as an option will add a check before each file to verify deletion  
• `rmdir <directory_name>` can also be used to delete empty directories
