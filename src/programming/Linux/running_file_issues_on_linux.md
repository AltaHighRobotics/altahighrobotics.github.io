
---
title: "Running file issues on linux"
parent: Linux
---
sources: [Unix & Linux](<https://unix.stackexchange.com/questions/79702/how-to-test-whether-a-file-uses-crlf-or-lf-without-modifying-it>) and [stackoverflow](<https://stackoverflow.com/questions/1552749/difference-between-cr-lf-lf-and-cr-line-break-types>)
___
A while ago I was having issues with my programs not running because they kept having a error that would look like this:
```
./pizza-paper.sh: line 3: $'\r': command not found
./pizza-paper.sh: line 10: $'\r': command not found
./pizza-paper.sh: line 47: syntax error near unexpected token `done'
'/pizza-paper.sh: line 47: `done  < "/home/$user/Documents/pizzapapers.txt"
```

And I found out that the reason why that was happening was that VSCode was somehow changing the file's "line feed" which is basically what the file uses to know when to move onto the next line (for syntax purposes).

The issue that I was having was the fact that VSCode so generously decided to do was change my normal linux LF (that's the name of the type of line feed) to window's version of line feed: CRLF, which is not compatible with linux scripts.

The quick fix for it is to just convert it over with a command that you install called `dos2unix` like this:
```bash
dos2unix <FILENAME>

# Example
# dos2unix pizzapaper.sh
```

That should fix the issue and make it so that the programs can be run normally again
