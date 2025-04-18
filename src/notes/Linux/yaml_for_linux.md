---
title: YAML
parent: Linux
---
# YAML for Linux

___
The package that I used to modify .yaml files was the most used and updated called `yq` that can be directly installed by that name with `sudo apt install yq`

## Creating yaml files/keys
To create a yaml file you will have to do it the normal way with:
`touch <FILE>.yaml`

In yaml's data system, it follows the same structure as dictionaries where you can store different values with certain "keys" that can be used to call/modify those values.

To create a new key + value, you need to both input them into the file.	
###### Note: You must include a value when declaring a key with yq
Lets start by making a `data1` key with value `69` into a file called `testing.yaml`

`yq -yi '.data1 = 69' testing.yaml     #testing.yaml must be an existing file`
###### Breakdown:
- `yq` will call upon the yaml editing command
- `-yi` will be used to WRITE to the file rather than READ
- `.data1` identifies the key in the dictionary that we are focusing on
- `= 69` sets the value of the `data1` key to the integer `69`
- `testing.yaml` is the file that will be edited

Now lets make a dictionary inside a dictionary, its just as simple as the first step, but instead you insert it into the main dictionary name:

`yq -yi '.mainDictionary.data1 = 69' testing.yaml`

Now, you might be thinking "What if I don't want to have a value assigned to the key?", not possible friend, at least not with `yq`, instead you have to echo it into the file like so:
Creating a new key called `data2` with no value

`echo "data2:" >> testing.yaml`

This will create an empty key, if you were to read it, it would print:
`data2: null`

## Deleting yaml elements
To delete an element, you need the name of the key

`yq -yi 'del(.mainDictionary.data1) testing.yaml`

This will delete the `data1` key and its value from inside the `mainDictionary` dictionary
If you don't include `-yi` it will just print out what it would be like IF the element was deleted

## Using yq with variables
The way to include variables is very weird and follows a similar style to the `sed` command

###### When using variables you MUST use double brackets ", not single '
If you want to do just a variable-named key, do:
`yq -yi ".mainDictionary.$VariableName = 69" testing.yaml`

For variable-named values, you must include an extra cover of double brackets:
`yq -yi '.mainDictionary.data1 = "$VariableName"' testing.yaml`

For BOTH variable-named keys and values, its a bit more weird
`yq -yi ".mainDictionary.$KeyName = \"$ValueName\"" testing.yaml`
As you can see I used `\"` around the value instead of just `"`, that is because you NEED double brackets for variable, but you also need it to prevent the quotes from closing
### Making comments:
Given a sample.yml file of:

```
a:
  # comment on key
  x: 3
  y: 4
```

You can use this command to print/add comments

```
yq '.a.x | key | headComment' sample.yml
```

will output the head comment of the key above it:

```
comment on key
```
