#!/bin/bash
# 把 $1 提供的图片转换成8位gray图，上传显示。
convert -rotate 90 -transverse -gravity Center -extent 1872x1404 $1 -depth 8 gray:- > /tmp/t3.png
curl -X POST -F "x=0" -F "y=0" -F "w=1872" -F "h=1404" -F "action=upload" -F "file=@/tmp/t3.png" http://10.3.165.240:9988/showupload
