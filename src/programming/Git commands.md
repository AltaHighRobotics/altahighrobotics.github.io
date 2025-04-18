Sources: [cheat sheet](<https://education.github.com/git-cheat-sheet-education.pdf>), [create sub-branch](<https://stackoverflow.com/questions/33652953/how-to-create-a-sub-branch>), [create remote branch](<https://stackoverflow.com/questions/1519006/how-do-i-create-a-remote-git-branch>), [compare commits](<https://stackoverflow.com/questions/17563726/how-can-i-see-the-changes-in-a-git-commit>), [compare branches](<https://stackoverflow.com/questions/43552274/how-can-i-diff-two-branches-in-github>)
___
## Downloading
### Download:
<u>alias</u> can also be the URL of the project
```bash
git clone [ALIAS]
```
### Download specific branch:
```bash
git clone --single-branch --branch=[BRANCH NAME] [ALIAS]
```

## Modifying online repository

#### Steps:
- Make changes to the files that you want to have modified
- Have the files added to the push list via `git add [FILENAME]`
- Commit the changes: `git commit`
- Push the changes to the online (remote) repository: `git push` (will push the current branch)
### Push

```
git push origin local-name:remote-name
```

## Creating a repository:
Creates a private repository on [github](<github.com/pizza2d1>)
```bash
gh repo create SIEMTesting --private
```
You can also make it public with `--public`
```bash
git remote add SIEMTesting https://github.com/pizza2d1/SIEMTesting
```

Will push current directory to the repo called "SIEMTesting" to the "master" branch, regardless of what the current directory is called
```
git push --set-upstream SIEMTesting master
```



## Creating branches
#### Local:
Just use the git branch command:
```bash
git branch [NEW_BRANCH_NAME]
```

#### Remote:
###### Create root branch:
```bash
# Create the branch and checkout into it
git checkout -b [NEW_BRANCH_NAME]

# Make the new branch a branch of origin
git push origin [NEW_BRANCH_NAME]

# Set git commits to go to that remote branch
git push --set-upstream origin [NEW_BRANCH_NAME]
```

###### Create sub-branch:
```bash
git push origin main:[NEW_BRANCH_NAME]
```
## Deleting branches
#### Local:
To delete the _**local**_ branch, use one of the following:

```
git branch -d <branch_name>
git branch -D <branch_name>
```

- The `-d` option is an alias for `--delete`, which only deletes the branch if it has already been fully merged in its upstream branch.
- The `-D` option is an alias for `--delete --force`, which deletes the branch "irrespective of its merged status." [Source: `man git-branch`]
- As of [Git v2.3](https://github.com/git/git/blob/master/Documentation/RelNotes/2.3.0.txt), `git branch -d` (delete) learned to honor the `-f` (force) flag.
- You will receive an error if you try to delete the currently selected branch.
#### Remote:
```
git push -d origin <branch_name> #will not auto-complete
# OR #
git push <remote_name> --delete <branch_name>
```
If you are deleting a sub-branch, you do not need to specify the parent branch

## Comparing Commits/Branches
#### Commits:
To compare between one commit and the other, you must get their commit hash:
```bash
git log # Will show you your list of commits and their hashes
```

Then you can use `git diff` to compare:
```bash
git diff [Commit1_HASH] [Commit2_HASH]

# Example:
git diff d9c448f746c27285981a00403959898006b57ddb 171ab48a6ffe190be5b4cfd8a0bc4cefd5639735
```

#### Branches:
Used diff with the branch names:
```bash
git diff branch_1 branch_2

# Can also be reversed to as git diff branch_2 branch_1
```

For example, if you want to find out what will happen when main gets merged **from** dev:
```bash
git diff main dev
```


# This is the extent of my knowledge for now




#git #commands/git