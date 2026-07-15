# RGB Recording & Video Conversion

- Recording `record_rgb` topic via rosbag command.
- Bag of commands for converting the rosbag results to mp4 or gif videos.

# 2. Setup (once)

## Packages

```bash
python3 -m venv ~/.venvs/rosbag2video
source ~/.venvs/rosbag2video/bin/activate
pip install rosbags opencv-python numpy static-ffmpeg
static_ffmpeg -version
git clone https://github.com/mlaiacker/rosbag2video ~/rosbag2video
cp ~/workspaces/movensys_ws/src/movensys-manipulator/tools/rosbag2video/rosbag2video.py ~/rosbag2video/rosbag2video.py
```

## Bashrc Configuration

```bash
# ROS2 based recording configuration
## Change *record_name* for each recording process
export record_name=basic_motion
export record_dir=~/recordings/${record_name}
alias ros_record='ros2 bag record -o ~/recordings/$record_name record_rgb'
alias activate_record='source ~/.venvs/rosbag2video/bin/activate'
alias bag2mp4='python3 ~/rosbag2video/rosbag2video.py -t /record_rgb -r 15 -o ${record_dir}/${record_name}.mp4 ~/recordings/${record_name}'
alias mp42gif='static_ffmpeg -i ${record_dir}/${record_name}.mp4 -vf "fps=15,scale=1280:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse" ${record_dir}/${record_name}.gif'
```

# 3. Useful commands

## Step 1. Start Record

- Ctrl + C will stop the recording.

```bash
ros_record
```

## Step 2. Activate venv for video

```bash
activate_record
```

## Step 3. bag → mp4

```bash
bag2mp4
```

## Step 4. mp4 → gif

```bash
mp42gif
```

# 4. Extensions

## 4-1. Speed up / slow down (x times)

`speed > 1` = faster, `speed < 1` = slower. Set `speed` once and reuse it.
`setpts=PTS/speed` divides each frame's timestamp, so `speed=2` plays twice as fast and `speed=0.5` plays at half speed.

### mp4

```bash
speed=2
static_ffmpeg -i ${record_dir}/${record_name}.mp4 -filter:v "setpts=PTS/$speed" -an ${record_dir}/${record_name}_${speed}.mp4
```

### gif

```bash
speed=2
static_ffmpeg -i ${record_dir}/${record_name}.gif -filter:v "setpts=PTS/$speed,split[a][b];[a]palettegen[p];[b][p]paletteuse" ${record_dir}/${record_name}_${speed}.gif
```

## 4-2. extract frames

Frames are written to `frames/%07d.png` (7-digit index) in the current directory.

### rosbag

```bash
python3 ~/rosbag2video/rosbag2video.py -t /record_rgb --save_images ${record_dir}/${record_name}
```

### mp4
```bash
static_ffmpeg -framerate 15 -i frames/%07d.png -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -pix_fmt yuv420p ${record_dir}/${record_name}.mp4
```

### gif
```bash
static_ffmpeg -framerate 15 -i frames/%07d.png -vf "fps=15,scale=1280:-1:flags=lanczos" ${record_dir}/${record_name}.gif
```