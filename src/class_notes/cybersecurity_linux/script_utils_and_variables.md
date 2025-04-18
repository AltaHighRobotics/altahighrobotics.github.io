---
title: Script Utilities and Variables
parent: Cybersecurity Linux
grand_parent: Class Notes
---
# Script Utilities and Variables
Cybersecurity Linux Lesson 3.1.3

___
### Common Script Utilities  
- `awk` scans files for specific patterns and extracts information based on those patterns  
- `sed` (stream editor) is designed for efficiently processing and transforming text streams
	- Allows for search and replace, insertion, deletion, etc.  
- `find` searches files and directories in a directory hierarchy  
- `xargs` takes inputs from a pipe or file and converts it into arguments for a specified command

### Additional Common Script Utilities  
- `grep` is used for searching patterns within text files  
- `egrep`/`grep -E` supports extended regular expressions  
- `tee` reads standard input and writes to both standard output and files simultaneously  
- `wc` (Word Count) counts lines, words, and characters in a file or input stream  
- `cut` extracts specific fields from each line of a file

### Remaining Common Script Utilities  
- `tr` translates or deletes characters in a stream of data  
- `head` displays the first few lines of a file  
- `tail` displays the last few lines of a file

### Environment Variables and Paths  
- `$PATH` specifies a colon-separated list of directories where the system looks for executable programs  
- `$SHELL` points to the user's default shell  
- `$?` Represents the exit status of the last executed command
	- 0 typically indicates success while non-zeros indicate errors or specific exit statuses

### Environment Variables and Paths  
- What does this command do:  
`echo $PATH | tr ':' '\n' | awk -F/ '{ print $NF }' | uniq`

### Relative and Absolute Paths  
- Relative paths describe the location of a file or directory in relation to the current working directory
	- Useful for navigating within the file system without specifying the full path
	- E.g. `./file`
- Absolute paths specify the complete path from the root directory to a file or directory
	- Provides an unambiguous reference
	- E.g. `/home/user/file`
