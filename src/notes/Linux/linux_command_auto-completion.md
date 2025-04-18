---
title: "Linux command auto-completion"
parent: Linux
---
# Linux Auto-completion
sources: [ask Ubuntu](<https://askubuntu.com/questions/68175/how-to-create-script-with-auto-complete>) (parameters) and [stackoverflow](<https://stackoverflow.com/questions/44441249/how-to-autocomplete-a-bash-commandline-with-file-paths>) (path completion)

___
## Use:
A way for my custom commands to have their parameters/arguments be auto-completed rather than having to type out the entire word, plus it can be used to show the different options that are available
## Simple options
For commands that only care about options rather than file paths
#### Example code:
```bash
_foo() {
	local cur prev opts
	COMPREPLY=()
	cur="${COMP_WORDS[COMP_CWORD]}"
	prev="${COMP_WORDS[COMP_CWORD-1]}"
	opts="--help --verbose --version"

	if [[ ${cur} == -* ]] ; then
		COMPREPLY=( $(compgen -W "${opts}" -- ${cur}) )
		return 0
	fi
}

## `foo` <tab> <tab> would show autocomplete above wordlist
complete -F _foo foo
```
###### (You place this in the /etc/bash_completion.d/ directory)
Or if you don't want to lose where it is by putting it in that obscure directory, you can source in in your .bashrc by adding this line:
```bash
source /path/to/your/autocomplete.sh
```

___
Or you can make one automatically with my `command_autocomp_maker.sh` custom command that I made so that I can easily transfer over my commands without there being an annoying bridge of config between them and since they are pretty simple themselves
## Complex options + file paths
source: [stack overflow](<https://stackoverflow.com/questions/44441249/how-to-autocomplete-a-bash-commandline-with-file-paths>)

For commands that are more specific, you can add options for parameters to include `/path/` completion, such as: `command1 --install /path/to/file` where you can set it to only have the `--install` parameter include `/path/` completion.
```bash
__command_name_autocomplete()
{
	local cur prev opts
	COMPREPLY=()
	cur="${COMP_WORDS[COMP_CWORD]}"
	prev="${COMP_WORDS[COMP_CWORD-1]}"
	opts="--help -h --install -i --run -r --rebuild -rb --show-running-containers -ps --stop -s --remove -rm --logs -l --bash -b --sass -css --unit-tests -t"
	containers="nginx php mysql mongo node"
	sass="watch"
	# By default, autocomplete with options
	if [[ ${prev} == command_name ]] ; then
		COMPREPLY=( $(compgen -W "${opts}" -- ${cur}) )	
		return 0
	fi
	# By default, autocomplete with options
	if [[ ${cur} == -* ]] ; then
		COMPREPLY=( $(compgen -W "${opts}" -- ${cur}) )
		return 0	
	fi
	
	# For --install and -i options, autocomplete with folder
	if [ ${prev} == --install ] || [ ${prev} == -i ] ; then
		COMPREPLY=( $(compgen -d -- ${cur}) )
		return 0	
	fi
	
	# For --stop --remove --logs and --bash, autocomplete with containers
	if [ ${prev} == --stop ] || [ ${prev} == -s ] || [ ${prev} == --remove ] || [ ${prev} == -rm ] || [ ${prev} == --logs ] || [ ${prev} == -l ] || [ ${prev} == --bash ] || [ ${prev} == -b ] ; then
		COMPREPLY=( $(compgen -W "${containers}" -- ${cur}) )
		return 0
	fi
	
	# For --sass and -css, complete with sass options
	if [ ${prev} == --sass ] || [ ${prev} == -css ] ; then
		COMPREPLY=( $(compgen -W "${sass}" -- ${cur}) )
		return 0
	fi
	
	# For --unit-tests and -t, complete with relative to command_name folder paths
	if [ ${prev} == --unit-tests ] || [ ${prev} == -t ] ; then
		COMPREPLY=( $(compgen -d -- ${cur}) )
		return 0
	fi

}

complete -o filenames -F __command_name_autocomplete command_name
```
