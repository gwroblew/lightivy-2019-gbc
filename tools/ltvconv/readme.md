ffmpeg -i "ralph.avi" -c:v cinepak -vf "scale=-4:176, crop=220:176" -q:v 2 -skip_empty_cb 1 -max_strips 7 -b:v 2000k -ar 22050 -ac 1 -c:a pcm_u8 wreckit.avi

ffmpeg -i ralph.avi -af "volumedetect" -vn -sn -dn -f null /dev/null
ffmpeg -i "ralph.avi" -vn -af "volume=12dB" -ar 22050 -ac 1 -c:a pcm_u8 wreckit_audio.wav
ffmpeg -i wreckit.avi -i wreckit_audio.wav -c:v copy -c:a copy -map 0:v:0 -map 1:a:0 wreckit2.avi

sudo mount -o loop sd.img /media/sdivy
sudo umount /media/sdivy
