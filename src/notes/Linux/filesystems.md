---
title: "Filesystems"
parent: Linux
---
# Filesystems

___
If you must extract and isolate additional storage from `sda` without the risk of data loss due to shrinking the primary drive which is never advised, then a safer approach would be to use `fallocate` to create a loop device.

`fallocate` is great for creating `swap` on systems but can also be used to great effect when partitioning is not an option.

First, create a mount point for the loop drive.

```
mkdir /NewDrive
```

Now, you can go ahead and create the loop device.

```
fallocate -l 5G NewStorage
```

The above command will create will create a file `NewStorage` with a file size of just under 5GB. You can confirm this with `ls -h`

Now that the storage is isolated rather than partitioned, you will need to create a filesystem with it.

```
mkfs.ext4 NewStorage
```

This will create a `ext4` filesystem. Now, you can mount it to the directory created earlier.

```
mount NewStorage /NewDrive
```

Your `lsblk` command should now return something like this

```
lsblk 
NAME MAJ:MIN RM   SIZE RO TYPE MOUNTPOINT
sda    8:0    0 298.1G  0 disk
sda1   8:1    0 298.1G  0 part /
loop0  7:0    0     5G  0 loop /NewDrive
```

And that is it without the risk of data loss or need for shrinking.

To see all active loop devices in use on your system, you can use `losetup`

```
losetup
NAME       SIZELIMIT OFFSET AUTOCLEAR RO BACK-FILE
/dev/loop0         0      0         1  0 /path/TO/NewDrive
```
