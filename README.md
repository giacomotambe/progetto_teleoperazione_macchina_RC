COMANDO AVVIO DOCKER (versione con GSstreamer, OpenCV e pyserial):

docker run -it --rm --network="host"  --device=/dev/ttyACM0:/dev/ttyACM0 --device=/dev/video4:/dev/video4 -v /home/pi/Desktop/workspace:/workspace ros/armv7:latest

COMANDO LETTURA STREAMING VIDEO SU PORTA UDP 5000:

gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink
