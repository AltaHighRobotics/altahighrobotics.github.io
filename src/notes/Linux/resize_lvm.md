<!-- ---
title: "Resize LVM"
parent: Linux
--- -->
# THIS IS NOT COMPLETE

___
## How to resize volumes with LVM

**Quick tips:**  

- To see the details of logical volume group use: `vgdisplay`
- To see the details of logical volumes use: `lvdisplay`
- To create new volumes, use: `lvcreate`
- To resize existing volumes, use: `lvresize`
- Use "--help" see quick help for the command:  `lvresize --help`

- It is easy to add extra space to an existing file system - no downtime required.
- It is much more difficult to shrink the file system - downtime IS required.
- Do not attempt to shrink a volume unless you really know what you are doing.
- More details here: [http://www.centos.org/docs/5/html/Cluster_Logical_Volume_Manager](http://www.google.com/url?q=http%3A%2F%2Fwww.centos.org%2Fdocs%2F5%2Fhtml%2FCluster_Logical_Volume_Manager&sa=D&sntz=1&usg=AFrqEzdaYf86KRv877W5Z2vbRJEqDIu_Sg)

### Before you begin:
- Run `df -h /` to make sure you have some free space available on your root volume. If your root volume is 100% full, you must clean it up by removing some files before you proceed.
- Run `vgdisplay` and note which volume groups you actually have.
- Run `lvdisplay` and note which logical volumes exist on your system.
- Adjust the examples below to match volume/group names.

## Example 1:
##### Add 10GB to "/" (root) partition
  
Add 10GB to "root" volume:
```bash
lvresize -L +10G /dev/VolGroup00/root  
```

Resize the filesystem that resides on that volume_:  
```bash
resize2fs /dev/VolGroup00/root  
```

Note: resize2fs may take a while to expand the file system on a large volume, so you may want to open a second terminal and monitor the progress with the following command:
```bash
watch df -h
```

## Example 2:
##### Set swap to 16GB

Disable all swaps:
```bash
swapoff -a
```

Set swap partition size to 16GB:
```bash
lvresize -L 16G /dev/VolGroup00/swap  
```

Resize the swap area to use new volume size:
```bash
mkswap /dev/VolGroup00/swap
```

Enable all swaps:
```bash
swapon -a
```

## Example 3:
##### Create a new volume "data" of 5GB and mount it as "/data"
  
Create the mount point:
```bash
mkdir /data
```

Create the volume:
```bash
lvcreate -L 5G -n data VolGroup00
```  

Create the filesystem:
```bash
mkfs.ext3 /dev/VolGroup00/data  
```

Add the following line to your /etc/fstab: (MIGHT BE WRONG)
```bash
/dev/VolGroup00/data /data ext3 defaults 0 2  
```

Mount the volume:
```bash
mount /data  
```
  
## Example 4: NOT COMPLETE
##### Reducing size of existing volume "test"
IMPORTANT:  
1. Reducing volume size CAN NOT be performed while file system is mounted and may take a LONG time.  
2. You will need to boot from a Rescue CD in order to be able to reduce the size of a volume which holds root file system.  
3. Improper reduction of a volume size WILL DESTROY YOUR DATA.  

Make sure you have a good backup and plenty of downtime scheduled, as it may take a while to shrink a filesystem.  
  
***You have been warned: proceed at your own risk.  
  
**First, make sure the file system has enough free space so it can be reduced:  
```bash
```
_root@server:~# **df -h /test/**_  
Filesystem Size  Used Avail Use% Mounted on
/dev/mapper/VolGroup00-__test
_5.0GÂ  139M 4.6GÂ Â  3% /test_

  
Note that even empty file system will use some space: 139M in our case.  
Proceed to unmount the file system.  
  

_root@server:~# **umount /test**_

  
Run a filesystem consistency check. This may take a while.  
  

_root@server:~# **fsck -f /dev/VolGroup00/test**_ _e2fsck 1.41.11 (14-Mar-2010)  
Pass 1: Checking inodes, blocks, and sizes  
Pass 2: Checking directory structure  
Pass 3: Checking directory connectivity  
Pass 4: Checking reference counts  
Pass 5: Checking group summary information  
__/dev/mapper/VolGroup00-__test__: 11/327680 files (0.0% non-contiguous), 55935/1310720 blocks_  
  

Find out file system block size  
  

_root@server:~# **tune2fs -l /dev/**__**VolGroup00**__**/test | grep 'Block size'**_  
_Block size:Â Â Â Â Â Â Â Â Â Â Â Â Â Â  4096_

  
Double check the minimum file system size in blocks:  
  

_root@server:~# **resize2fs -P /dev/**__**VolGroup00**__**/test**_  
_resize2fs 1.41.11 (14-Mar-2010)_  
_Estimated minimum size of the filesystem: 34477_

  
In this example, the minimum logical volume size required is: 34477*4096 = 141217792 bytes.  
Here, you can make calculations to figure out what your file system final size should be, proceed with caution. It is much safer to shrink a file system to its minimum size instead.  
  
Depending on the current file system size, usage and layout, this may take a LONG time.  
  

_root@server:~# **resize2fs -M**_ _**/dev/**__**VolGroup00**__**/test**__  
resize2fs 1.41.11 (14-Mar-2010)  
Resizing the filesystem on /dev/raid1/test to 34477 (4k) blocks.  
The filesystem on /dev/raid1/test is now 34477 blocks long.  
_

  
**MAKE SURE THAT YOUR PLANNED NEW VOLUME SIZE IS GREATER THAN THE SIZE OF THE FILESYSTEM.  
**Keep in mind that LVM allocates space to volumes on Physical Extent (PE) granularity, so you may not be able to get the exact size you want.  
  

_root@server:~# **vgdisplay**_  
_--- Volume group ---_  
_VG Name_ _VolGroup00_  
_System ID_  
_FormatÂ Â Â Â Â Â Â Â Â Â Â Â Â Â Â  lvm2_  
_Metadata AreasÂ Â Â Â Â Â Â  1_  
_Metadata Sequence NoÂ  44_  
_VG AccessÂ Â Â Â Â Â Â Â Â Â Â Â  read/write_  
_VG StatusÂ Â Â Â Â Â Â Â Â Â Â Â  resizable_  
_MAX LVÂ Â Â Â Â Â Â Â Â Â Â Â Â Â Â  0_  
_Cur LVÂ Â Â Â Â Â Â Â Â Â Â Â Â Â Â  20_  
_Open LVÂ Â Â Â Â Â Â Â Â Â Â Â Â Â  20_  
_Max PVÂ Â Â Â Â Â Â Â Â Â Â Â Â Â Â  0_  
_Cur PVÂ Â Â Â Â Â Â Â Â Â Â Â Â Â Â  1_  
_Act PVÂ Â Â Â Â Â Â Â Â Â Â Â Â Â Â  1_  
_VG SizeÂ Â Â Â Â Â Â Â Â Â Â Â Â Â  930.50 GiB_  
_PE SizeÂ Â Â Â Â Â Â Â Â Â Â Â Â Â  4.00 MiB_  
_Total PEÂ Â Â Â Â Â Â Â Â Â Â Â Â  238207_  
_Alloc PE / SizeÂ Â Â Â Â Â  212966 / 831.90 GiB_  
_FreeÂ  PE / SizeÂ Â Â Â Â Â  25241 / 98.60 GiB_  
_VG UUIDÂ Â Â Â Â Â Â Â Â Â Â Â Â Â  LazRIZ-d8rU-2bwB-uI7o-6Amt-BH0w-oFxNh_

In our example, PE size is 4MB, we will have to round up the planned volume size to next larger multiple of 4.  
Just to be on a safe side, we strongly recommend you adding some extra "padding", so in this example we will shrink the volume down to 200M, which is greater than BOTH 139M used as reported by df and 141M as calculated in blocks.  
  

_root@server:~# **lvresize -L 200M**_ _**/dev/**__**VolGroup00**__**/test**__  
WARNING: Reducing active logical volume to 200.00 MiB  
THIS MAY DESTROY YOUR DATA (filesystem etc.)  
Do you really want to reduce test? [y/n]: **y**  
Reducing logical volume test to 200.00 MiB  
Logical volume test successfully resized  
_

  
Now, grow the file system to use all of the space available in a logical volume:  
  

_root@server:~# **resize2fs**_ _**/dev/**__**VolGroup00**__**/test**__resize2fs 1.41.11 (14-Mar-2010)  
Resizing the filesystem on_ _/dev/__VolGroup00__/test_ _to 51200 (4k) blocks.  
The filesystem on_ _/dev/__VolGroup00__/test_ _is now 51200 blocks long._

Mount the file system and check the new size:

_root@server:~# **mount**_ **_/dev/__VolGroup00__/test_ _/test_**  
_root@server:~# **df -h /test**_  
_FilesystemÂ Â Â Â Â Â Â Â Â Â Â  SizeÂ  Used Avail Use% Mounted on_  
_/dev/mapper/__VolGroup00__-test_  
_196M 131MÂ Â  56MÂ  71% /test_
