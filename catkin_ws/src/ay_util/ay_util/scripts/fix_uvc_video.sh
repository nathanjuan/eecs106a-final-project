#!/bin/bash
#\file    fix_uvc_video.sh
#\brief   Fix the issue of UVC video for multiple webcams.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.01, 2018

sudo rmmod uvcvideo
sudo modprobe uvcvideo quirks=128
