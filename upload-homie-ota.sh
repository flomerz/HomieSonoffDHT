UPLOAD_URL=http://ha:9080/upload
DESCRIPTION=${PWD##*/}_`date +"%Y%m%d%H%M%S"`

curl -F upload=@.pioenvs/build/firmware.bin -F description=$DESCRIPTION $UPLOAD_URL
