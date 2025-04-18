
---
title: "Flashing drives"
parent: Linux
---
###### Create a compressed image file with a working image file

```bash
dd if=/dev/sdb | gzip > backup.img.gz
```

###### Flash a drive with compressed image file

```bash
cat backup.img.gz | gunzip | dd of=/dev/sdb
```


## ROBOTICS VERSION:
For flashing a sd card to the roborio
```bash
gunzip -c /mnt/mydrive/img.gz > /dev/sda
```




#commands/dd #commands/gzip
