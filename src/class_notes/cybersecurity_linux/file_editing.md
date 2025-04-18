---
title: File Editing
parent: Cybersecurity Linux
grand_parent: Class Notes
---
# File Editing
Cybersecurity Linux Lesson 1.2.1

___
### Text Editing  
- Text editing is an important skill when using a Linux system  
- Whether changing a configuration file or writing a script, having a grasp on more than one text editor in Linux will be valuable  
- The text editors shown in this lesson are:
	- nano
	- vi(m)  
- These text editors require different levels of skill, but both have solid features

### Nano Editor  
- Nano is the more user-friendly of the two editors  
- In a terminal, type nano and the name of the file to create to start using this editor  
- Similar to other word-processing applications in that you can immediately begin typing without commands  
- Comes with its own shortcuts and lists them on the bottom of the editor so that you don’t have to remember them

### Nano Shortcuts  

| Shortcut | Alternative Keys | Description |
| -------- | --- | ------------------------------------- |
| Ctrl + G | F1 | Get Help, displays a help text box  |
| Ctrl + X | F2 | Close the current buffer / Exit from nano (Nano then asks if you want to save with a Y or N option) |
| Ctrl + O | F3 | Write the current buffer (or marked region) to disk / Save |
| Ctrl + R | Ins | Insert another file into the current buffer (or into a new buffer) |  
| Ctrl + W | F6 | Search forward for a string or regular expression  |
| Ctrl + \ | M-R* | Replace a string or regular expression  |
| Ctrl + K | F9 | Cut current line (or marked region) / Like Ctrl + X in word processing  |
| Ctrl + U | F10 | Paste the contents of cutbuffer / Like Ctrl + V in word processing  |
| Ctrl + J | F4 | Justify the current paragraph  |
| Ctrl + T | F12 | Invoke the spell checker, if available  |
| Ctrl + C | F11 | Display the position of the cursor  |
| Ctrl + _ | M-G* | Go to line and column number  |

Notice the alternative keys for Replace and Go To Line are M-R and M-G. The M represents Esc or Alt or the Meta key on your keyboard depending on how it is set up. On the CYBER.ORG Range, that M would stand for Alt. So, those commands would look like Alt + R and Alt + G.

### Vi or Vim Editor  
- When you are ready for a more frustrating... uh, powerful text editing experience, vi(m) editor is the choice for you.  
- As the previous line suggest, vi(m) editor is not novice friendly. But since you can’t get at something until you try it, let’s look at vi(m).  
- Vi(m) is the more popular editor for scripting and programming.  
- Just like with nano, to use this editor you type vi (or vim depending on the distribution) and then the name of the file.
###### The Three Modes Vi or Vim Editor  
| Command Mode                                                                                                                                                  | Insert Mode                                                                                                                                                                                 | Ex Mode                                                                                        |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| Sometimes called normal mode, this is how vi(m) looks when first opened. It will only accept commands. Some common commands will be listed on the next slide. | By pressing i on the keyboard, you’ll notice the word INSERT at the bottom of the screen. This means you are now able to enter text just in nano. Press Esc to switch back to command mode. | This mode is sometimes called colon commands because commands here are preceded by a colon (:) |



### Vi(m) Shortcuts

###### Command Mode
| Keystroke | Description |
| --- | --- |
| h | Move cursor left one character. |
| l | Move cursor right one character.  |
| j | Move cursor down one line. |
| k | Move cursor up one line.  |
| w | Move cursor forward one word to front of next  |
| e | Move cursor to end of current word.  |
| b | Move cursor backward one word.  |
| ^ | Move cursor to beginning of line. |
| $ | Move cursor to end of line.  |
| gg | Move cursor to file’s first line. |  
| G |Move cursor to file’s last line.  |

###### Ex Mode
| Keystroke | Description |
| --- | --- |
| :x | Write buffer to file and quit editor.  |
| :wq | Write buffer to file and quit editor.  |
| :wq! | Write buffer to file and quit editor.  |
| :w | Write buffer to file and stay in editor.  |
| :w! | Write buffer to file and stay in editor.  |
| :q | Quit editor without writing buffer to file. |
| :q! | Quit editor without writing buffer to file. |

### Stream Editors
- Once files have been created, stream editors can change the way text looks on a standard output – the screen  
- Stream editors can range from simple to complex and range from changing words in text to helping sort large text outputs into more manageable chunks  
- The stream editors we will discuss are:
	- `printf`
	- `sed`  
	- `awk`

### printf  
• `printf` will show on the screen (your standard output) whatever you input into the command using the formatting you ask of it.  
• The printf command works like this: `printf FORMAT [ARGUMENT]`  
• So, with the command `printf “%s\n” “Hello, World!”`, let’s take a look at what we are asking the command to do.

###### printf Format Options  
| Format  | Description |
| --- | --- |
| %c | Display the first ARGUMENT character.  |
| %d | Display the first ARGUMENT as a decimal integer number.  |
| %f | Display the ARGUMENT as a floating-point number.  |
| %s | Display the ARGUMENT as a character string.  |
| \% | Display a percentage sign.  |
| \” | Display a double quotation mark. |  
| \\ | Display a backslash.  |
| \f | Include a form feed character.  |
| \n | Include a newline character.  |
| \r | Include a carriage return.  |
| \t | Include a horizontal tab.  |

Okay, so based on the chart, what will this command do?  
It prints the string ‘Hello, World!’ and then puts the command line on a new line, of course!

### sed 
• `sed` stands for stream editor  
• `sed` can be used on text files with one pass as the text “streams” through this utility  
• In that single pass, `sed` looks for patterns in the text as dictated by the user  
• The command looks like: `sed [OPTIONS] [SCRIPT]... [FILENAME]`
• Much like the `printf` command, `sed` can very simply print text to the screen, but make changes according to requests
###### sed Commonly Used Options  
• When using sed with a file, we’ll need some options.  
• Here are the most common options.  
• Option i (-i) will actually save the changes we make with sed to the file we are using so that we don’t have to go back and do that later. 

| Short | Long | Description |
| --- | ----------------- | --- |
| -e script | --expression=script | Add commands in script to text processing. The script is written as part of the sed command. |
| -f script | --file=script | Add commands in script to text processing. The script is a file.  |
| -r | --regexp-extended | Use extended regular expressions in script.  |
| -i | --in-place | Edits file in place. |
 
### awk  
• `sed` and `awk` have a lot in common in the way they can change text  
• `awk` is more powerful since it is a programming language for completing formatted reports for large data sets  
• A big difference is that `awk` treats each item separated by a space as a separate field  
• The command looks like: `awk [OPTIONS] [PROGRAM]... [FILENAME]`
• `awk` comes in a variety of versions. When GNU project rewrote it, it became `gawk`. Either way you type it, many distribution will still put from `gawk`

###### awk Commonly Used Options  

| Short | Long | Description |
| --- | ----------------- | --- |
| -F d | --field-separator d | Specify the delimiter that separates the data file’s fields |
| -f file | --file=file | Use program in file for text processing. |
| -s | --sandbox | Execute awk program in sandbox mode. |
• Option F (-F) allows us to tell awk what the delimiter will be  
that separates fields.  
• If we wanted to print just the users in the /etc/shadow file for  
instance, we would want to indicate that the delimiter of that  
file is the : (colon) between each field.
