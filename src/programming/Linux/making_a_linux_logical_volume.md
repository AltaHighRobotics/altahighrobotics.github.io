
---
title: "Making a linux logical volume"
parent: Linux
---
[Source](<https://medium.com/@yhakimi/lvm-how-to-create-and-extend-a-logical-volume-in-linux-9744f27eacfe>)
___
# Start:
Make sure that your disk has a clean partition that you are able to convert into a usable disk, aka allow the LVM (logical volume manager) to use your storage.

# Option 1. Using entire disk:
First, you want to delete all partitions and previous data that is on the drive, you CANNOT directly convert a drive into a logical volume while keeping the data, as it using formatting to allow for drive expansion.

### Delete partitions
We will use the `fdisk` tool that comes with most linux distributions to modify your disk.

First, list all your drives to make sure that you are selecting the right drive:
```bash
sudo fdisk -l
```

Then, its time to wipe the disk of all partitions (replace sdXX with the drive you choose):
```bash
sudo fdisk /dev/sdXX
```

This will open the fdisk menu, you should see
`Command (m for help):`
at the bottom of the terminal

### Quick note
If you got a red error saying that the drive is running, MAKE SURE TO UNMOUNT THE DEVICE. If you cannot unmount it conventionally, run:
```bash
sudo umount -l /dev/sdXX
```
(if you are not sure where the mount is located, use gnome-disks)

Run these commands (the letters, also one at a time):
```bash
# Deletes your partition (might need to run multiple times)
Command (m for help): d

# Creates a new clean partition
Command (m for help): n
# Press enter through all of these
Partition Number: (1-128, default 1): <Press Enter>
First Sector (2048-XXXXX, default 2048): <Press Enter>
Last Sector, +/-sectors or +/-size{K,M,G,T,P} (2048-XXXXX, default XXXXX): <Press Enter>

# Edits the kind of partition that it will be, in this case we want 8e (LVM)
Command (m for help): t
Selected Partition: 1
Partition type or alias (type L to list all): 8e

# Write this to the disk so that it will be saved
Command (m for help): w
```

# Option 2. Extend Partitions WORKING ON
If you want to use unused parts of your primary/drive or want to just use extra space on other drives without compromising their data, you can instead use that extra space as logical volume styled partitions

```bash
# Deletes your partition (might need to run multiple times)
Command (m for help): d

# Creates a new clean partition
Command (m for help): n
# Press enter through all of these
Partition Number: (1-128, default 1): <Press Enter>
First Sector (2048-XXXXX, default 2048): <Press Enter>
Last Sector, +/-sectors or +/-size{K,M,G,T,P} (2048-XXXXX, default XXXXX): <Press Enter>

# Edits the kind of partition that it will be, in this case we want 8e (LVM)
Command (m for help): t
Selected Partition: 1
Partition type or alias (type L to list all): 8e

# Write this to the disk so that it will be saved
Command (m for help): w
```





You now have a partitioned disk that can be used for LVM purposes

## Creating a Logical Volume
Now that you have a free partition that can be used for a LVM, you need to make it into a physical drive

###### Creating a physical drive

```bash
sudo pvcreate /dev/sdXX
```

To verify that it worked, run:

```bash
sudo pvdisplay
```

###### Creating the volume group

This will make a volume group (vg) named vg-data, you can use a different name instead:
```bash
sudo vgcreate vg-data /dev/sdXX

# Check to see that it worked:
sudo vgs
```

###### Creating the logical volume

Will create the logical volume that will soon be mounted to your system so that you can use it, you can replace `lv-data` with another name, just remember that `vg-data` is the right name.

```bash
# Will use all of the free space in the volume group to add to the logical volume
sudo lvcreate --name lv-data -l 100%FREE vg-data
```

You now have a logical volume! This can be added to easily so that you can expand your storage, even if you have processes running on the logical volume (really nice).

## Adding LV to file-system
Now that you have a logical volume, you'll notice that you can't save any data to it quite yet, in order to do that you need to make/add a file-system

###### Option 1. Making your LV into a file-system
If you have not already made a accessible logical volume, this is how to make one:
```bash
# "mkfs" means make filesystem, this creates a filesystem based
# on the entirety of your logical volume.
sudo mkfs.xfs /dev/vg-data/lv-data

# If "mkfs.xfs" doesn't work, just use "mkfs" which will use ext4 instead of xfs
```

Now to mount the file-system to your system:
```bash
# You can replace the "/data" folder with another name if you wish
sudo mkdir /data
sudo mount /dev/vg-data/lv-data /data
```

###### Option 2. Adding a new LV to existing file-system
If you already have a file-system that uses LVs and want to add another LV to it, you don't need to make a new file-system for it, instead do:
```bash
resize2fs /dev/vg-data/lv-data
```

This will try and resize the file-system to the new size of your logical volume, assuming you added to it.

# End


#commands/fdisk #commands/umount #commands/mount #commands/pvcreate #commands/vgcreate #commands/lvcreate #commands/resize2fs
