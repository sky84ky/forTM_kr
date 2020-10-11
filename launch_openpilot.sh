#!/usr/bin/bash

if [ ! -f "/data/no_ota_updates" ]; then
    /usr/bin/touch /data/no_ota_updates
fi

ALIAS_CHECK1=$(/usr/bin/grep tune_pid /system/comma/home/.bash_profile)
ALIAS_CHECK2=$(/usr/bin/grep tune_indi /system/comma/home/.bash_profile)
ALIAS_CHECK3=$(/usr/bin/grep tune_lqr /system/comma/home/.bash_profile)

if [ "$ALIAS_CHECK1" == "" ]; then
    sleep 3
    mount -o remount,rw /system
    echo "alias tp='/data/openpilot/tune_pid.sh'" >> /system/comma/home/.bash_profile
    mount -o remount,r /system
fi
if [ "$ALIAS_CHECK2" == "" ]; then
    sleep 3
    mount -o remount,rw /system
    echo "alias ti='/data/openpilot/tune_indi.sh'" >> /system/comma/home/.bash_profile
    mount -o remount,r /system
fi
if [ "$ALIAS_CHECK3" == "" ]; then
    sleep 3
    mount -o remount,rw /system
    echo "alias tl='/data/openpilot/tune_lqr.sh'" >> /system/comma/home/.bash_profile
    mount -o remount,r /system
fi

if [ ! -f "/system/fonts/opensans_regular.ttf" ]; then
    sleep 3
    mount -o remount,rw /system
  	cp -rf /data/openpilot/selfdrive/assets/fonts/opensans* /system/fonts/
    cp -rf /data/openpilot/kyd/fonts.xml /system/etc/fonts.xml
    chmod 644 /system/etc/fonts.xml
  	chmod 644 /system/fonts/opensans*
    mount -o remount,r /system
fi

if [ ! -f "/data/KRSet" ]; then
    setprop persist.sys.locale ko-KR
    setprop persist.sys.local ko-KR
    setprop persist.sys.timezone Asia/Seoul
    /usr/bin/touch /data/KRSet
fi

export PASSIVE="0"
exec ./launch_chffrplus.sh