---
title: Account Creation and Deletion
parent: Cybersecurity Linux
grand_parent: Class Notes
---
# Account Creation and Deletion
Cybersecurity Linux Lesson 2.2.1

___
### Creating User Accounts  
To create a new user account, we use the command `useradd` followed by the username.  
Format:
```bash
sudo useradd <options> <new username>
```

For example:  
```bash
sudo useradd john
```

### Adding More Information  
You can change or add info to a user account with the `usermod` command. Format: 
```bash
sudo usermod <options> <username>  
```

For example:  
```bash
sudo usermod –c “John Smith” john  
```
This will set the user’s comment, which is often a full name, to John Smith.  

Or
```bash
sudo usermod –g Sales john
```
This will add the john account into the Sales group.

### Adding Groups  
The `groupadd` command in Linux is used to create a new user group on the system.  
Format: 
```bash
sudo groupadd <options> <new group name> 
``` 

For example:  
```bash
sudo groupadd mygroup  
```
Groups can be used to manage users with similar permissions or rights.

### Modifying Groups  
Groups can by modified with the `groupmod` command.  
Format: 
```bash
sudo groupmod <options> <group name>  
```

For example:  
```bash
sudo groupmod –n supergroup mygroup  
```
This would change the name of an existing group from "mygroup" to “supergroup”.

### Deleting User Accounts and Groups  
To delete a user account, use the `userdel` command followed by the username. For example:  
```bash
sudo userdel john  
```

To delete the account plus their directories use the `–r` option:  
```bash
sudo userdel -r john  
```

Similarly groupdel deletes entire groups.  
```bash
sudo groupdel mygroup
```

### Check User Information  
There are commands you can use to quickly check system information about a user.  
The `id` command displays the user's unique ID, their groups and other user account properties. Use the following command:  
```bash
id john  
```

The `who` command displays list of logged-in users.  
The `w` command provides detailed information about logged in users including the time they logged-in and the processes they are running
