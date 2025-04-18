___
**Assigning Variables

StringName="Words"
IntName=Number#
ArrayName=()   or ArrayName=(item1, item2)

**Calling Variables

$StringName returns "Words"
$IntName returns Number#
$ArrayName returns **first** item of array
${ArrayName[index]} returns array item
___
### Strings

Add to String:
`NewString="$String <NEW_CONTENT>"

Modify String:
```bash
String="Hello World"
NewString=${String:2:5}   #${ STRING_BEING_MODIFIED : STARTING_POS : OFFSET }` 
```
String output:  "Hello World"
NewString output: "lo Wo"
___
### Arrays
Are piss-poor, I hate them

First item in Array:
`$ArrayName`

Second item in Array:
`${ArrayName[1]}`

All items of an Array:
`${ArrayName[@]}`

Number of items in an Array (int):
`$(#ArrayName[@]}`

^^^^ Awful



Bash colors: https://misc.flogisoft.com/bash/tip_colors_and_formatting

```bash
Black        0;30     Dark Gray     1;30
Red          0;31     Light Red     1;31
Green        0;32     Light Green   1;32
Brown/Orange 0;33     Yellow        1;33
Blue         0;34     Light Blue    1;34
Purple       0;35     Light Purple  1;35
Cyan         0;36     Light Cyan    1;36
Light Gray   0;37     White         1;37
```

Examples:
RED='\033[1;31m'
GREEN='\033[0;32m'

Icons: https://github.com/sebastiencs/icons-in-terminal?tab=readme-ov-file#how-it-works