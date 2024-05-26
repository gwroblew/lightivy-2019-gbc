#!/bin/bash

# This util was written for ESP8266 Device Hive firmware
# But it can be used for any other projects
# Author Khabarov Nikolay
# License is MIT

set -e

if [ "$#" -lt 1 ]; then
  echo Util for generating binary firmware images for ESP8266
  echo Please specify compliled binary in arg
fi

TARGETELF=$1
BUILDDIR=$(dirname "$1")
FWFILE="oc.bin"
if [ "$#" -gt 1 ]; then
  FWFILE="$2"
fi

if [[ $(uname) == *"MINGW"* ]]; then
  CROSS_COMPILE="c:/Espressif/xtensa-lx106-elf/bin/xtensa-lx106-elf-"
else
  CROSS_COMPILE="../xtensa-lx106-elf/bin/xtensa-lx106-elf-"
fi
OBJCOPY=${CROSS_COMPILE}objcopy
NM=${CROSS_COMPILE}nm

write_hex32() {
  echo -n -e  $(echo $1 | sed -E 's/(..)(..)(..)(..).*/\\x\4\\x\3\\x\2\\x\1/') >> $FWFILE
}

write_addr() {
  write_hex32 $($NM $TARGETELF | grep "$1")
}

file_size() {
  res=$(stat -c %s "$@" 2> /dev/null)
  if [ $? -ne 0 ]; then 
    res=$(stat -f %z "$@")
  fi
  echo $res
}

write_zeros() {
  printf %*s $1 | tr ' ' '\000' >> $FWFILE
}

write_section() {
  PAD=4
  FILESIZE=$(file_size "$@")
  PADSIZE=$(( ($PAD - $FILESIZE % $PAD) % $PAD ))
  BYTESWITHPAD=$(( $FILESIZE + $PADSIZE ))
  write_hex32 $(printf %08X $BYTESWITHPAD)
  cat "$@" >> $FWFILE
  write_zeros $PADSIZE
}

write_checksum() {
  PAD=16
  FILESIZE=$(( $(file_size $FWFILE) + 1))
  PADSIZE=$(( ($PAD - $FILESIZE % $PAD) % $PAD ))
  write_zeros $PADSIZE
  r=0xef;
  for c in $(od -vt u1 $@ | sed 's/[^ ]*//'); do 
    r=$(($c ^ $r))
  done
  echo -n -e $(printf "\\\x%02X" $r) >> $FWFILE
}

echo Extract sections...
$OBJCOPY --only-section .text -O binary $TARGETELF $BUILDDIR/text.bin
$OBJCOPY --only-section .irom0.text -O binary $TARGETELF $BUILDDIR/irom0.bin
rm -f $FWFILE

echo Writing .text section...
cat $BUILDDIR/irom0.bin >> $FWFILE
cat $BUILDDIR/text.bin >> $FWFILE

echo Done
echo "Firmware file is $FWFILE, size is $(file_size $FWFILE) bytes"
