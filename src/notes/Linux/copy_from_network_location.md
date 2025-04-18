---
title: "Copy from Network Location"
parent: Linux
grand_parent: Notes
---

# Copy from Network Location
source: [Unix and Linux](<https://unix.stackexchange.com/questions/106480/how-to-copy-files-from-one-machine-to-another-using-ssh>)

___
Syntax:
```
scp <source> <destination>
```

To copy a file from `B` to `A` while logged into `B`:
```
scp /path/to/file username@[A's IP_ADDRESS]:/path/to/destination
```

To copy a file from `B` to `A` while logged into `A`:
```
scp username@[B's IP_ADDRESS]:/path/to/file /path/to/destination
```

Example:
```bash
scp pizza2d1@192.168.0.77:/home/pizza2d1/ObbyBackup /home/pizza2d1/
```
Will copy the ObbyBackup file from the NAS home directory to my home directory
