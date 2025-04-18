---
title: Shell Script Elements
parent: Cybersecurity Linux
grand_parent: Class Notes
---
# Shell Script Elements
Cybersecurity Linux Lessons 3.1.1 and 3.1.2

___
# <u><span style="color:rgb(0, 176, 80)">Part 1</span></u>
### Loops  
• A “while” loop executes a set of commands if a specified condition is true  
• A “for” loop iterates over a sequence, like a range of numbers, and performs commands  
• An “until” loop executes a set of commands if the specified condition is false

### Conditionals  
Control structures that allow one to make decisions and execute different sets of commands based on true/false conditions  
• An if statement executes a set of commands based on the evaluation of a condition  
• Switch/Case provides multiple possible execution paths based on the value of an expression

### Shell Parameters  
A feature in shell scripting that allows one to manipulate and expand the values of variables  
• Extract substrings, perform pattern matching, etc.  
• Globbing matches filenames with patterns  
• Brace expressions generate arbitrary strings using curly braces

### Shell Comparisons  
Essential for making decisions based on the values of variables or the success/failure of commands 
• Using double parentheses, arithmetic operations can be performed  
• Strings can be compared with operators like == (the same) or != (not the same)

### Boolean Logic  
• Shell scripting doesn't have a native boolean type but still uses boolean logic  
• Users can combine conditions using logical operators, && for AND, || for OR

### Shell Variables and Search and Replace  
• Used to store and manipulate data 
• Act as placeholders that can be referenced or modified within a script  
• If a piece of text needs to have a part replaced, "search and replace" does that

### Regular Expressions  
• Regex or regexp are powerful patterns used for matching character combinations with strings  
• Wildly used for tasks like string manipulation, text parsing, and pattern matching

# <u><span style="color:rgb(0, 176, 80)">Part 2</span></u> 
### Shell Script Elements  
• The | (Pipe) connects the output of one command as the input to another command  
• This helps chain commands together  
• The example below generates a fortune and uses the tool cowsay to display the fortune

### Or, Oar, or Ore  
• The || (OR) command executes the command following it ONLY IF the preceding command fails/returns an error  
• Often used in control flow  
• An alternative to an if-then-else command

### Redirection  
• The > (Output Redirection) redirects the standard output to a file, overwriting it if the file already exists  
• The >> (Also Output Redirection) appends the standard output to the end of a file, rather than overwrite it  
• The < (Input Redirection) redirects the input from a file to a command  
• The << (Here Document) allows multiple lines to be put into a command

### And, &, and &&  
• The &> combines standard output and standard error to a specified file, overwriting the file if it already exists  
• The && (AND) command executes the following following it ONLY IF the preceding command succeeds
	- Understanding Boolean logic and truth tables makes understanding these operators a little easier

### Standard Standards  
• STDOUT is the standard output stream where a program writes its regular output  
• STDERR is the standard error stream where a program writes its error messages

### Exit Codes  
• Also known as return codes or status codes  
• Numeric values returned by a command or program to indicate the result of its execution
	- 0 typically means success  
	- Nonzero values typically indicate an error or some kind of failure  
• The OS or shell keeps track of the exit code so other scripts or processes can proceed

### Shell Built-In Commands  
• Built into the shell itself  
• More efficient because they don't require a separate process  
• read reads input from a user or file  
• echo prints text or variables to the standard output  
• source execute commands from a file in the current shell environment
