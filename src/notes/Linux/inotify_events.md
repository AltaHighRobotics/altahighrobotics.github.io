---
title: "inotify events"
parent: Linux
---
# inotify command events

___
## OPEN
A watched file or a file within a watched directory was opened.

## ACCESS 
A watched file or a file within a watched directory was read from.

## MODIFY
A watched file or a file within a watched directory was written to.

## CREATE
A file or directory was created within a watched directory.

## DELETE
A file or directory within a watched directory was deleted.

## ATTRIB
The metadata of a watched file or a file within a watched directory was modified.  This includes timestamps, file permissions, extended attributes etc.

## CLOSE_WRITE
A watched file or a file within a watched directory was closed, after being opened in writable mode.  This does not necessarily imply the file was written to.

## CLOSE_NOWRITE
A watched file or a file within a watched directory was closed, after being opened in read-only mode.

## CLOSE
A watched file or a file within a watched directory was closed, regardless of how it was opened.  Note that this is actually implemented simply by listening for both close_write and close_nowrite, hence all close events received will be output as one of these, not CLOSE.

## MOVED_TO
A file or directory was moved into a watched directory.  This event occurs even if the file is simply moved from and to the same directory.

## MOVED_FROM
A file or directory was moved from a watched directory.  This event occurs even if the file is simply moved from and to the same directory.

## MOVE
A file or directory was moved from or to a watched directory.  Note that this is actually implemented simply by listening for both moved_to and moved_from, hence all move events received will be output as one or both of these, not MOVE.

## MOVE_SELF
A watched file or directory was moved. After this event, the file or directory is no longer being watched.

## DELETE_SELF
A  watched  file  or directory was deleted.  After this event the file or directory is no longer being watched.  Note that this event can occur even if it is not explicitly being listened for.

## UNMOUNT
The filesystem on which a watched file or directory resides was unmounted.  After this event the file or directory is no longer being watched.  Note that this  event can occur even if it is not explicitly being listened to.
