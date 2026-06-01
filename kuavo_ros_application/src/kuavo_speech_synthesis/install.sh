#!/bin/bash
INSTALL_DIR=/opt/lejurobot
APP_NAME=kuavo-wifi-announce
TTS_MODEL_OSS_URL="https://kuavo.lejurobot.com/models/tts_model-speech_sambert-hifigan_tts_zhida_zh-cn_16k.tar.gz"
TTS_MODEL_MD5_OSS_URL="https://kuavo.lejurobot.com/models/tts_model-speech_sambert-hifigan_tts_zhida_zh-cn_16k.tar.gz.md5"

start_time=$(date +%s)

function echo_info() {
  local message=$1
  echo -e "$message"
}

function echo_warn() {
  local message=$1
  echo -e "\033[33m$message\033[0m"
}

function echo_error() {
  local message=$1
  echo -e "\033[31m$message\033[0m"
}

function echo_success() {
  local message=$1
  echo -e "\033[32m$message\033[0m"
}

function check_root() {
  if [ $(id -u) != "0" ]; then
    echo_error "You must be root to run this script, please use root to install."
    exit 1
  fi
}

function usage() {
  echo "Usage: $0 --robot-name \"your_robot_name\""
  echo "Options:"
  echo "  --robot-name <name>  Set the name of the robot (required)."
  echo "  -h                  Show this help message and exit."
}

################### main ##################

check_root
robot_name=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --robot-name|-r)
        if [[ -n $2 && $2 != -* ]]; then
            robot_name="$2"
            shift
            shift
        else
            echo "Error: --robot-name requires a non-empty value."
            usage
            exit 1
        fi
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        echo "Error: Unknown option: $1"
        usage
        exit 1
        ;;
  esac
done
# @@@ DEPENDENCIES!
echo_warn "✅ Installing dependencies..."
# Check for processes holding dpkg lock
if sudo lsof /var/lib/dpkg/lock-frontend 2>/dev/null; then
    echo_warn "Another process is holding the dpkg lock. Waiting for it to finish..."
    echo_warn "Processes holding the lock:"
    sudo lsof /var/lib/dpkg/lock-frontend
    retry_count=5
    while sudo lsof /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
        sleep 1
        echo_warn "Still waiting for dpkg lock to be released..."
        echo_warn "Current processes holding the lock:"
        sudo lsof /var/lib/dpkg/lock-frontend
        retry_count=$((retry_count - 1))
        if [ $retry_count -le 0 ]; then
            echo_error "Timeout waiting for dpkg lock to be released after multiple attempts"
            exit 1
        fi
    done
fi

sudo apt-get update || { echo_error "apt-get update failed, exiting..."; exit 1; }
sudo apt-get install -y network-manager iw portaudio19-dev|| { echo_error "apt-get install failed, exiting..."; exit 1; }

# @@@ PYTHON ENV!
if ! command -v python3 &> /dev/null; then
    echo_error "Python3 is not installed. Please install Python3 and try again."
    exit 1
fi

python_version=$(python3 --version | cut -d ' ' -f 2 | cut -d '.' -f 1-2)
sudo apt-get install -y python${python_version}-venv || { echo_error "ERROR: apt-get install python${python_version}-venv failed, exiting..."; exit 1; }

VENV_PATH="$INSTALL_DIR/$APP_NAME/venv"
if [ -f "$VENV_PATH/bin/activate" ]; then
    echo_info "Python virtual environment already exists at $VENV_PATH"
else
    rm -rf "$VENV_PATH"
    echo_warn "✅ Creating Python3 venv at $VENV_PATH..."
    python3 -m venv "$VENV_PATH"
    if [ $? -ne 0 ]; then
        echo_error "Failed to create Python virtual environment at $VENV_PATH"
        exit 1
    fi
fi

source $VENV_PATH/bin/activate
# TODO check pip install success！
pip install networkx==2.8.8 # fix ERROR: Package 'networkx' requires a different Python: 3.8.10 not in '>=3.9'
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip install modelscope
pip install -r requirements.txt
pip install kantts -f https://modelscope.oss-cn-beijing.aliyuncs.com/releases/repo.html 
pip install rospkg catkin_pkg
pip install empy==3.3.4

#@@@ COPY-FILES!
echo_warn "✅ Copying files..."
if [ ! -d "$INSTALL_DIR" ]; then
  mkdir -p "$INSTALL_DIR"
fi
mkdir -p "$INSTALL_DIR/$APP_NAME/"
mkdir -p "$INSTALL_DIR/$APP_NAME/bin/"
mkdir -p "$INSTALL_DIR/$APP_NAME/config/"
mkdir -p "$INSTALL_DIR/$APP_NAME/data/model/"

# download tts-model 
echo_warn "✅ Downloading TTS model..."
tts_model_file="/tmp/tts_model-speech_sambert-hifigan_tts_zhida_zh-cn_16k.tar.gz"
if [ -f "$tts_model_file" ]; then
    echo_warn "TTS model file already exists, skipping download."
else
    wget $TTS_MODEL_OSS_URL -O $tts_model_file
    if [ $? -ne 0 ]; then
        echo_error "Failed to download the TTS model!"
        exit 1
    fi
fi

# md5 check
echo_info "✅ Checking MD5 of the TTS model..."
wget -q -O - $TTS_MODEL_MD5_OSS_URL > /tmp/tts_model.md5
if [ $? -ne 0 ]; then
    echo_error "Failed to download the MD5 file from $TTS_MODEL_MD5_OSS_URL"
    exit 1
fi
expected_md5=$(cat /tmp/tts_model.md5)
if [ -z "$expected_md5" ]; then
    echo_error "Expected MD5 is empty!"
    exit 1
fi
actual_md5=$(md5sum $tts_model_file | awk '{print $1}')
if [ "$actual_md5" != "$expected_md5" ]; then
    echo_error "MD5 checksum verification failed for the TTS model, Please try again."
    rm -rf $tts_model_file
    exit 1
fi
echo_success "🎉 MD5 checksum verification passed for the TTS model!"

# extract tts model
tar -xvf $tts_model_file -C "$INSTALL_DIR/$APP_NAME/data/model/"
if [ $? -ne 0 ]; then
    echo_error "Failed to extract the TTS model file"
    rm -f $tts_model_file
    exit 1
fi


# 更换/opt/lejurobot下所有文件的用户组为 kuavo:kuavo
echo_info "✅ 更改文件所有权为 kuavo:kuavo..."
if [ -d "/opt/lejurobot" ]; then
    chown -R kuavo:kuavo /opt/lejurobot
    if [ $? -ne 0 ]; then
        echo_error "更改文件所有权失败！"
        exit 1
    fi
    echo_success "🎉 成功更改 /opt/lejurobot 下所有文件的所有权为 kuavo:kuavo"
else
    echo_warn "目录 /opt/lejurobot 不存在，跳过所有权更改"
fi


#@@@ END! 
end_time=$(date +%s)
elapsed_time=$(( (end_time - start_time) / 60 )) # Convert to minutes
elapsed_time="${elapsed_time%.*}"

echo_success "\n🚀🚀🚀 Installation completed in $elapsed_time minutes."
echo_success "\n🚀🚀🚀 Success! Please reboot your system to complete the installation.\n"

####################
# @TEST
####################
